/*
 * nv_journal.hpp
 *
 * NvJournal: a handful of small values kept in FLASH, rewritten often,
 * and readable after any power loss - the storage class an MCU with no
 * EEPROM does not have and every application still wants.
 *
 * WHAT IT IS FOR, AND WHAT IT IS NOT. util/nv_heap.hpp opens by naming
 * two needs and one storage class each: "a handful of small values,
 * rewritten often - a calibration offset, a mode, a counter - belong in
 * the EEPROM", and "a table belongs somewhere else". NvHeap is the
 * second one. This header is the FIRST one, on a part where the first
 * sentence's EEPROM does not exist. It is not a heap and it is not a
 * log: there is no address handed out, no block, no growth. There are
 * ids, and each id has a current value.
 *
 * THE TWO HALVES. The journal's region is split into two halves that
 * ping-pong WHOLESALE. Entries are APPENDED into the active half, one
 * write cell at a time and never rewritten; when an append plus the
 * reserve (below) no longer fits, a COLLECTION copies the latest value
 * of every live id into the freshly erased other half and erases the
 * old one. That is the same atomicity argument as the heap's map pair,
 * one size up: nothing is ever programmed twice between erases, the old
 * half rules until the new data is entirely down, and a power loss
 * anywhere costs at most the entry being written - never history.
 *
 * SEQUENCE NUMBERS DECIDE, CRC JUDGES. Every entry carries a
 * monotonically increasing 32-bit sequence number and a CRC-16 over its
 * own header and payload. The current value of an id is the
 * highest-seq entry with a valid CRC, wherever it lies; the ACTIVE half
 * is the one holding the highest-seq valid entry of all. Those two
 * rules alone resolve every state a power loss can leave:
 *
 *  - torn mid-append: the entry fails its CRC and the previous value of
 *    that id still rules;
 *  - torn mid-collection, before the source half was erased: both
 *    halves hold entries, the destination is the one with the newer
 *    entries, and the collection is RESUMED - ids whose latest still
 *    lives in the source are copied over, then the source is erased;
 *  - torn while erasing the source: every live id is already in the
 *    destination, so the resume copies nothing and simply finishes the
 *    erase;
 *  - torn while erasing the DESTINATION (a half is one or more erase
 *    units, so that erase is not one hardware operation either): the
 *    survivors there are older than everything in the source, so the
 *    source is the newer half and is treated as the destination - and
 *    since every live id was copied into it at the previous collection,
 *    the resume again copies nothing and erases the stale half.
 *
 * None of those is a special case in the code: they are what "newer
 * half is the destination, drain the other into it, then erase the
 * other" does.
 *
 * MOUNT IS READ-ONLY. Booting costs no erase cycle and no program
 * cycle: mount() walks both halves, builds the id index in RAM, and
 * reports what it found - including how many torn entries it stepped
 * over, which is the atomicity working and not an error. A collection
 * left unfinished by a power loss is NOTED at mount and completed by
 * the next ordinary save(), never by the boot path.
 *
 * THE PANIC RESERVE. An ordinary save() collects EARLY: it leaves the
 * active half with room for at least one more maximum-size entry in
 * cells that are already erased. save_reserved() spends exactly that
 * room - no collection, no erase, one bounded polled program - which is
 * what makes it legal from a panic handler, where interrupts are masked
 * for good and an erase would be an unbounded wait on a dying supply.
 * JournalPanic is that reporter; the room it needs is guaranteed by the
 * static_assert on the geometry, not by hoping.
 *
 * WEAR. A save costs one program cell. A collection costs one erase of
 * every unit of both halves plus one program cell per live id. So the
 * erase budget is spent once per (half capacity / average entry size)
 * saves, and the two halves wear evenly by construction.
 *
 * WHAT IS DELIBERATELY NOT HERE. No iteration over ids in write order
 * (this is a set of values, not a log). No compaction policy knob. No
 * delete verb: an application that wants an id to mean "absent" writes
 * a payload that says so, because a tombstone entry would cost the same
 * cell and buy nothing. And no unification with util/nv_record.hpp -
 * see docs/design/nv-journal.md for why the two spellings stay apart.
 *
 * ONE CONSEQUENCE WORTH KNOWING: if the flash holds more distinct ids
 * than `max_ids` - an older firmware wrote them, or the parameter was
 * lowered - the surplus is not indexed at mount and is dropped at the
 * next collection, in silence. That is the right behaviour for a
 * deliberately shrunken configuration and the wrong one for a typo, and
 * nothing here can tell the two apart.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <span>
#include <type_traits>

#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "util/crc.hpp"
#include "util/nv_heap.hpp"   // FlashZone and the FlashMedia concept

namespace brio {

/// What mount() found.
enum class NvJournalStatus : uint8_t {
    unmounted = 0,  ///< mount() has not run yet
    ok,             ///< at least one valid entry was read
    empty,          ///< both halves are blank (or hold nothing that verifies)
    bad_geometry,   ///< the media's zones do not contain the journal's
                    ///< region - REFUSED, nothing was written
};

/// Why the last save() refused. Readable through last_error().
enum class NvJournalError : uint8_t {
    none = 0,
    not_mounted,    ///< mount() has not run, or it refused
    too_big,        ///< the payload is larger than an entry can ever be
    too_many_ids,   ///< a NEW id, and max_ids are already live
    no_room,        ///< the half cannot hold it even after a collection
    media,          ///< the flash refused a program
    gc_failed,      ///< the flash refused an erase during a collection
};

/// One live id, as the index holds it. Public so a status console can
/// walk what is stored without a second copy of the arithmetic.
struct NvJournalEntry {
    uint32_t seq;      ///< the sequence number of the entry in force
    uint32_t address;  ///< where that entry's header starts
    uint8_t id;        ///< the application's name for the value
    uint8_t length;    ///< payload bytes
};

/**
 * A journal of small values over a FlashMedia.
 *
 *   Media       the flash (util/nv_heap.hpp's concept, unchanged: two
 *               granularities, cells written once between erases, a
 *               runtime zone report).
 *   max_ids     how many distinct ids may be live at once. It sizes the
 *               RAM index and is what the geometry assertion is about.
 *   max_payload bytes in the largest value. Fixes the entry size the
 *               reserve is measured in.
 *   half_pages  erase units in ONE half. The journal occupies the top
 *               2 x half_pages erase units of the media, anchored to
 *               Media::flash_end - to the SILICON, like the heap's map
 *               home, so no image can be built over it by accident.
 *
 * The constructor is trivial and constexpr; nothing touches the flash
 * until mount().
 */
template <FlashMedia Media, uint8_t max_ids = 8, uint8_t max_payload = 32,
          uint8_t half_pages = 2>
class NvJournal {
public:
    static constexpr uint32_t erase_size = Media::erase_size;
    static constexpr uint32_t write_cell = Media::write_cell;
    static constexpr uint32_t flash_end = Media::flash_end;

    /// The entry's on-flash layout, little-endian throughout:
    ///   0  magic u16   2  id u8      3  length u8
    ///   4  seq u32     8  crc u16   10  reserved u16
    ///   12 payload[length], then 0xFF padding to a whole write cell.
    /// The CRC covers bytes 0..7 of the header and then the payload -
    /// everything that is not the checksum itself and not padding.
    static constexpr uint16_t magic = 0x4A4Eu;   // 'N','J'
    static constexpr uint16_t header_bytes = 12;
    static constexpr uint16_t off_magic = 0;
    static constexpr uint16_t off_id = 2;
    static constexpr uint16_t off_length = 3;
    static constexpr uint16_t off_seq = 4;
    static constexpr uint16_t off_crc = 8;
    static constexpr uint16_t off_reserved = 10;
    static constexpr uint16_t crc_covered_header = 8;

    /// The largest entry, in bytes and in cells: a max_payload value
    /// with its header, rounded up to whole program units.
    static constexpr uint32_t max_entry_bytes =
        (static_cast<uint32_t>(header_bytes) + max_payload + write_cell - 1u) /
        write_cell * write_cell;
    static constexpr uint32_t max_entry_cells = max_entry_bytes / write_cell;

    static constexpr uint32_t half_bytes =
        static_cast<uint32_t>(half_pages) * erase_size;
    static constexpr uint32_t half_cells = half_bytes / write_cell;

    /// The two halves, low first. The region is the top 2 x half_bytes
    /// of the media.
    static constexpr uint32_t journal_home = flash_end - 2u * half_bytes;
    static constexpr uint32_t half_base(uint8_t h) {
        return journal_home + static_cast<uint32_t>(h) * half_bytes;
    }

    /// The room save() always leaves behind: one maximum-size entry, in
    /// cells that are already erased. save_reserved() spends it.
    static constexpr uint32_t reserve_cells = max_entry_cells;

    static_assert(max_ids >= 1, "a journal with no ids is not a journal");
    static_assert(max_payload >= 1, "a value of no bytes is not a value");
    static_assert(half_pages >= 1,
                  "a half is at least one erase unit: erasing it must not "
                  "take the other half down with it");
    static_assert(erase_size >= write_cell && erase_size % write_cell == 0,
                  "an erase unit must be a whole number of program units");
    static_assert(flash_end % erase_size == 0,
                  "the journal is anchored to flash_end, which must be an "
                  "erase-unit boundary");
    static_assert(2u * half_bytes <= flash_end,
                  "the journal would swallow the whole part");
    // The reserve is a GUARANTEE, so the arithmetic behind it is
    // checked and not assumed: after a collection the half holds at
    // most max_ids maximum-size entries, the save being made adds one
    // more, and one more still must remain free for save_reserved().
    static_assert((static_cast<uint32_t>(max_ids) + 2u) * max_entry_cells <=
                      half_cells,
                  "one half cannot hold max_ids maximum-size entries plus the "
                  "one being written plus the panic reserve: raise half_pages, "
                  "or lower max_ids or max_payload");

    /// What mount() found, in full.
    struct MountReport {
        NvJournalStatus status = NvJournalStatus::unmounted;
        uint8_t live = 0;         ///< ids with a current value
        uint8_t torn = 0;         ///< entries stepped over for a bad CRC
        uint8_t active = 0;       ///< which half is being appended to
        bool collect_pending = false;  ///< a collection was interrupted
        uint32_t seq = 0;         ///< the highest sequence number seen
        uint32_t used_cells = 0;  ///< cells occupied in the active half

        bool mounted() const {
            return status == NvJournalStatus::ok ||
                   status == NvJournalStatus::empty;
        }
    };

    constexpr NvJournal() = default;

    NvJournal(const NvJournal&) = delete;
    NvJournal& operator=(const NvJournal&) = delete;

    // ---- mounting ----------------------------------------------------------

    /**
     * Walk both halves, build the index, and open the journal. Costs no
     * erase and no program: this is the boot path.
     *
     * A bad_geometry verdict leaves the journal CLOSED - load() and
     * save() both refuse - because the only thing a journal whose home
     * is occupied by the running image can do is damage.
     */
    const MountReport& mount() {
        report_ = MountReport{};
        count_ = 0;
        seq_ = 0;
        active_ = 0;
        used_ = 0;
        collect_pending_ = false;
        mounted_ = false;
        error_ = NvJournalError::none;

        if (!geometry_ok()) {
            report_.status = NvJournalStatus::bad_geometry;
            return report_;
        }

        uint32_t high_water[2] = {0, 0};
        uint32_t half_max_seq[2] = {0, 0};
        bool half_has[2] = {false, false};
        uint8_t torn = 0;

        for (uint8_t h = 0; h < 2; ++h) {
            scan_half(h, high_water[h], half_max_seq[h], half_has[h], torn);
        }

        report_.torn = torn;
        if (half_has[0] || half_has[1]) {
            active_ = half_max_seq[1] > half_max_seq[0] ? 1 : 0;
            collect_pending_ = half_has[0] && half_has[1];
            report_.status = NvJournalStatus::ok;
        } else {
            // Nothing verifies anywhere. Append into whichever half is
            // less dirty; a wholly blank part gives 0.
            active_ = high_water[1] < high_water[0] ? 1 : 0;
            report_.status = NvJournalStatus::empty;
        }
        used_ = high_water[active_];
        mounted_ = true;
        report_.live = count_;
        report_.active = active_;
        report_.collect_pending = collect_pending_;
        report_.seq = seq_;
        report_.used_cells = used_;
        return report_;
    }

    const MountReport& report() const { return report_; }
    bool mounted() const { return mounted_; }
    /// The highest sequence number issued so far.
    uint32_t sequence() const { return seq_; }
    /// Which half is being appended to.
    uint8_t active_half() const { return active_; }
    /// Cells occupied in the active half.
    uint32_t used_cells() const { return used_; }
    /// Cells still free in the active half.
    uint32_t free_cells() const { return used_ > half_cells ? 0u
                                                            : half_cells - used_; }
    /// True while the reserve save_reserved() spends is intact. It is
    /// the invariant every completed save() restores; it is false only
    /// between a mount that found a torn collection and the next save.
    bool reserve_intact() const { return free_cells() >= reserve_cells; }
    /// A collection was interrupted by a power loss and will be
    /// completed by the next save().
    bool collect_pending() const { return collect_pending_; }

    /// The live ids, in no particular order.
    uint8_t count() const { return count_; }
    const NvJournalEntry& entry(uint8_t i) const { return index_[i]; }
    /// Why the last save() refused.
    NvJournalError last_error() const { return error_; }

    // ---- reading -----------------------------------------------------------

    /// True when this id has a current value.
    bool has(uint8_t id) const { return slot_of(id) != max_ids; }

    /**
     * Copy the current value of `id` into `dst` and return how many
     * bytes that was. Nothing when the id has no value, or when `dst`
     * is too small to take all of it - a short read would be a lie,
     * not a convenience.
     */
    std::optional<uint8_t> load(uint8_t id, std::span<uint8_t> dst) const {
        if (!mounted_) {
            return std::nullopt;
        }
        const uint8_t s = slot_of(id);
        if (s == max_ids || dst.size() < index_[s].length) {
            return std::nullopt;
        }
        Media::read(index_[s].address + header_bytes,
                    dst.subspan(0, index_[s].length));
        return index_[s].length;
    }

    /// The typed twin: the whole object or nothing. The stored length
    /// must match exactly, so a struct that changed size reads as
    /// absent rather than as garbage.
    template <typename T>
    std::optional<T> load(uint8_t id) const {
        static_assert(std::is_trivially_copyable_v<T>,
                      "a value that must be memcpy-able to flash and back");
        static_assert(sizeof(T) <= max_payload,
                      "this type is larger than the journal's max_payload");
        if (!mounted_) {
            return std::nullopt;
        }
        const uint8_t s = slot_of(id);
        if (s == max_ids || index_[s].length != sizeof(T)) {
            return std::nullopt;
        }
        T value{};
        Media::read(index_[s].address + header_bytes,
                    std::span<uint8_t>(reinterpret_cast<uint8_t*>(&value),
                                       sizeof(T)));
        return value;
    }

    // ---- writing -----------------------------------------------------------

    /**
     * Make `data` the current value of `id`.
     *
     * This is the ORDINARY path: it finishes an interrupted collection,
     * collects when the entry plus the reserve no longer fit, and
     * leaves the reserve intact behind it. It may therefore erase, and
     * it may take as long as an erase takes - which is why it is not
     * the path a panic handler uses.
     *
     * False, with last_error() saying which, when the payload can never
     * fit, when the id would be one too many, when the media refuses a
     * program or an erase, or when the journal is not mounted.
     */
    bool save(uint8_t id, std::span<const uint8_t> data) {
        if (!mounted_) {
            error_ = NvJournalError::not_mounted;
            return false;
        }
        if (data.size() > max_payload) {
            error_ = NvJournalError::too_big;
            return false;
        }
        error_ = NvJournalError::none;
        if (collect_pending_ && !collect()) {
            return false;
        }
        if (slot_of(id) == max_ids && count_ >= max_ids) {
            error_ = NvJournalError::too_many_ids;
            return false;
        }
        const uint32_t need = cells_for(static_cast<uint8_t>(data.size()));
        // Collect EARLY: what must still fit afterwards is this entry
        // AND the reserve.
        if (used_ + need + reserve_cells > half_cells && !collect()) {
            return false;
        }
        if (used_ + need > half_cells) {
            error_ = NvJournalError::no_room;
            return false;
        }
        return append(id, data);
    }

    /// The typed twin.
    template <typename T>
    bool save(uint8_t id, const T& value) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "a value that must be memcpy-able to flash and back");
        static_assert(sizeof(T) <= max_payload,
                      "this type is larger than the journal's max_payload");
        return save(id, std::span<const uint8_t>(
                            reinterpret_cast<const uint8_t*>(&value),
                            sizeof(T)));
    }

    /**
     * Make `data` the current value of `id` WITHOUT collecting and
     * WITHOUT erasing: one program into cells the last ordinary save
     * left erased for exactly this.
     *
     * Bounded, polled and safe from a panic handler. It spends the
     * reserve, so a second call in the same failure may find nothing
     * left - which is honest: after a panic the program is not going to
     * write anything else either.
     */
    bool save_reserved(uint8_t id, std::span<const uint8_t> data) {
        if (!mounted_) {
            error_ = NvJournalError::not_mounted;
            return false;
        }
        if (data.size() > max_payload) {
            error_ = NvJournalError::too_big;
            return false;
        }
        error_ = NvJournalError::none;
        if (slot_of(id) == max_ids && count_ >= max_ids) {
            error_ = NvJournalError::too_many_ids;
            return false;
        }
        const uint32_t need = cells_for(static_cast<uint8_t>(data.size()));
        if (used_ + need > half_cells) {
            error_ = NvJournalError::no_room;
            return false;
        }
        return append(id, data);
    }

    /// The typed twin.
    template <typename T>
    bool save_reserved(uint8_t id, const T& value) {
        static_assert(std::is_trivially_copyable_v<T>,
                      "a value that must be memcpy-able to flash and back");
        static_assert(sizeof(T) <= max_payload,
                      "this type is larger than the journal's max_payload");
        return save_reserved(id, std::span<const uint8_t>(
                                     reinterpret_cast<const uint8_t*>(&value),
                                     sizeof(T)));
    }

    /**
     * Copy every live value into the other half and erase this one -
     * or, when a collection was interrupted, finish that one instead.
     * save() calls it when it has to; an application may call it to
     * spend the erase time at a moment of its own choosing.
     */
    bool collect() {
        if (!mounted_) {
            error_ = NvJournalError::not_mounted;
            return false;
        }
        // The destination is the half holding the newest entries: on an
        // ordinary collection that is the fresh one, on a resumed one it
        // is where the copies already went.
        const uint8_t dest = collect_pending_
                                 ? active_
                                 : static_cast<uint8_t>(1u - active_);
        const uint8_t source = static_cast<uint8_t>(1u - dest);
        if (!collect_pending_) {
            // The destination is the idle half: erasing it cannot lose
            // anything, and until the source is erased at the end this
            // whole operation is one a power loss can be resumed from.
            if (!erase_half(dest)) {
                error_ = NvJournalError::gc_failed;
                return false;
            }
            active_ = dest;
            used_ = 0;
        }

        // Everything whose current entry does not already live in the
        // destination is copied over, with a fresh sequence number.
        // append() rewrites the id's index slot in place, so the loop
        // index stays valid and a copied id is skipped if seen again.
        for (uint8_t i = 0; i < count_; ++i) {
            if (in_half(index_[i].address, dest)) {
                continue;
            }
            uint8_t buf[max_payload];
            const uint8_t len = index_[i].length;
            if (len != 0) {
                Media::read(index_[i].address + header_bytes,
                            std::span<uint8_t>(buf, len));
            }
            if (!append(index_[i].id, std::span<const uint8_t>(buf, len))) {
                // Both halves hold entries and the destination's are the
                // newer ones: exactly the state a power loss here would
                // have left, and the next collect() resumes it.
                collect_pending_ = true;
                return false;
            }
        }
        if (!erase_half(source)) {
            collect_pending_ = true;
            error_ = NvJournalError::gc_failed;
            return false;
        }
        collect_pending_ = false;
        return true;
    }

private:
    // ---- geometry ----------------------------------------------------------

    /// The media must contain the journal's region: every band it
    /// reports has to be inside the part, the right way up, and none of
    /// them may have a FLOOR above the journal home - a floor is the
    /// first byte the journal may use, so a floor inside the region
    /// means the running image's own bytes are sitting where an entry
    /// would be programmed.
    static bool geometry_ok() {
        for (uint8_t i = 0; i < Media::zone_count; ++i) {
            const FlashZone z = Media::zones()[i];
            if (z.ceiling > flash_end || z.floor > z.ceiling) {
                return false;
            }
            if (z.floor > journal_home) {
                return false;
            }
        }
        return true;
    }

    static constexpr uint32_t cells_for(uint8_t len) {
        return (static_cast<uint32_t>(header_bytes) + len + write_cell - 1u) /
               write_cell;
    }

    static bool in_half(uint32_t addr, uint8_t h) {
        return addr >= half_base(h) && addr < half_base(h) + half_bytes;
    }

    uint8_t slot_of(uint8_t id) const {
        for (uint8_t i = 0; i < count_; ++i) {
            if (index_[i].id == id) {
                return i;
            }
        }
        return max_ids;
    }

    // ---- the walk ----------------------------------------------------------

    /**
     * Parse one half cell by cell. A well-formed entry with a good CRC
     * updates the index; one with a bad CRC is counted and stepped
     * over; anything unrecognizable advances by a single cell, so a
     * torn write can never hide the entries behind it.
     *
     * `high_water` comes back as the first cell that may be programmed:
     * past the last entry parsed AND past the last cell that is not
     * still blank.
     */
    void scan_half(uint8_t h, uint32_t& high_water, uint32_t& max_seq,
                   bool& any, uint8_t& torn) {
        const uint32_t base = half_base(h);
        uint32_t cell = 0;
        uint32_t parsed_end = 0;
        // The smallest entry there is. Stopping when even that cannot
        // start here keeps every header read inside the half - which
        // matters on a target where one byte past the array is not
        // memory at all.
        const uint32_t min_cells = cells_for(0);
        while (cell + min_cells <= half_cells) {
            uint8_t head[header_bytes];
            Media::read(base + cell * write_cell,
                        std::span<uint8_t>(head, header_bytes));
            const uint8_t len = head[off_length];
            const uint32_t cells = cells_for(len);
            if (load16(head + off_magic) != magic || len > max_payload ||
                cell + cells > half_cells) {
                ++cell;
                continue;
            }
            uint16_t crc = crc16(head, crc_covered_header);
            uint8_t buf[max_payload];
            if (len != 0) {
                Media::read(base + cell * write_cell + header_bytes,
                            std::span<uint8_t>(buf, len));
                crc = crc16(buf, len, crc);
            }
            if (crc != load16(head + off_crc)) {
                ++torn;
                cell += cells;
                parsed_end = cell;
                continue;
            }
            const uint32_t seq = load32(head + off_seq);
            remember(head[off_id], len, seq, base + cell * write_cell);
            any = true;
            if (seq > max_seq) {
                max_seq = seq;
            }
            if (seq > seq_) {
                seq_ = seq;
            }
            cell += cells;
            parsed_end = cell;
        }
        // Nothing may be programmed over a cell that is not blank, even
        // one no entry claims.
        uint32_t dirty_end = 0;
        for (uint32_t c = half_cells; c > parsed_end; --c) {
            if (!blank_cell(base + (c - 1u) * write_cell)) {
                dirty_end = c;
                break;
            }
        }
        high_water = parsed_end > dirty_end ? parsed_end : dirty_end;
    }

    static bool blank_cell(uint32_t addr) {
        uint8_t buf[write_cell];
        Media::read(addr, std::span<uint8_t>(buf, write_cell));
        for (uint32_t i = 0; i < write_cell; ++i) {
            if (buf[i] != 0xFFu) {
                return false;
            }
        }
        return true;
    }

    /// Highest seq wins, everywhere and always.
    void remember(uint8_t id, uint8_t len, uint32_t seq, uint32_t addr) {
        for (uint8_t i = 0; i < count_; ++i) {
            if (index_[i].id != id) {
                continue;
            }
            if (seq > index_[i].seq) {
                index_[i] = NvJournalEntry{seq, addr, id, len};
            }
            return;
        }
        if (count_ < max_ids) {
            index_[count_++] = NvJournalEntry{seq, addr, id, len};
        }
    }

    // ---- writing one entry ---------------------------------------------------

    /// Build the whole entry - header, payload, 0xFF padding - and
    /// program it as ONE call, so the media sees a single append and
    /// the cells inside it go down in order.
    bool append(uint8_t id, std::span<const uint8_t> data) {
        const uint8_t len = static_cast<uint8_t>(data.size());
        const uint32_t cells = cells_for(len);
        if (used_ + cells > half_cells) {
            error_ = NvJournalError::no_room;
            return false;
        }
        uint8_t image[max_entry_bytes];
        for (uint32_t i = 0; i < cells * write_cell; ++i) {
            image[i] = 0xFFu;
        }
        const uint32_t seq = seq_ + 1u;
        store16(image + off_magic, magic);
        image[off_id] = id;
        image[off_length] = len;
        store32(image + off_seq, seq);
        store16(image + off_reserved, 0xFFFFu);
        for (uint8_t i = 0; i < len; ++i) {
            image[header_bytes + i] = data[i];
        }
        uint16_t crc = crc16(image, crc_covered_header);
        crc = crc16(image + header_bytes, len, crc);
        store16(image + off_crc, crc);

        const uint32_t addr = half_base(active_) + used_ * write_cell;
        if (!Media::program(addr, std::span<const uint8_t>(
                                      image, cells * write_cell))) {
            error_ = NvJournalError::media;
            return false;
        }
        seq_ = seq;
        used_ += cells;
        remember_new(id, len, seq, addr);
        return true;
    }

    /// The index after a successful append: this id's entry is now that
    /// one, whatever it was before.
    void remember_new(uint8_t id, uint8_t len, uint32_t seq, uint32_t addr) {
        for (uint8_t i = 0; i < count_; ++i) {
            if (index_[i].id == id) {
                index_[i] = NvJournalEntry{seq, addr, id, len};
                return;
            }
        }
        if (count_ < max_ids) {
            index_[count_++] = NvJournalEntry{seq, addr, id, len};
        }
    }

    bool erase_half(uint8_t h) {
        for (uint8_t p = 0; p < half_pages; ++p) {
            if (!Media::erase(half_base(h) +
                              static_cast<uint32_t>(p) * erase_size)) {
                return false;
            }
        }
        return true;
    }

    // ---- little-endian, by hand ---------------------------------------------

    static void store16(uint8_t* at, uint16_t v) {
        at[0] = static_cast<uint8_t>(v);
        at[1] = static_cast<uint8_t>(v >> 8);
    }
    static void store32(uint8_t* at, uint32_t v) {
        at[0] = static_cast<uint8_t>(v);
        at[1] = static_cast<uint8_t>(v >> 8);
        at[2] = static_cast<uint8_t>(v >> 16);
        at[3] = static_cast<uint8_t>(v >> 24);
    }
    static uint16_t load16(const uint8_t* at) {
        return static_cast<uint16_t>(at[0] | (static_cast<uint16_t>(at[1]) << 8));
    }
    static uint32_t load32(const uint8_t* at) {
        return static_cast<uint32_t>(at[0]) |
               (static_cast<uint32_t>(at[1]) << 8) |
               (static_cast<uint32_t>(at[2]) << 16) |
               (static_cast<uint32_t>(at[3]) << 24);
    }

    // ---- state --------------------------------------------------------------

    MountReport report_{};
    NvJournalEntry index_[max_ids]{};
    uint32_t seq_ = 0;
    uint32_t used_ = 0;
    uint8_t count_ = 0;
    uint8_t active_ = 0;
    bool collect_pending_ = false;
    bool mounted_ = false;
    NvJournalError error_ = NvJournalError::none;
};

/**
 * A panic Reporter that writes the breadcrumb into a journal, and the
 * boot-side verb that takes it back out - util/persistent_panic.hpp's
 * shape over flash instead of over an EEPROM record.
 *
 *   inline brio::NvJournal<brio::RwweeJournalZone, 6, 32> journal;
 *   using Panic = brio::JournalPanic<journal, 0>;
 *   ...
 *   brio::panic<P, Panic>(PanicCode::assert_failed, 7);   // at the fault
 *   ...
 *   auto old = Panic::take();                             // at boot
 *
 * The journal is reached through a REFERENCE template parameter because
 * a journal is an object with a RAM index, not a static store: there is
 * one per media and the application owns it.
 *
 * report() uses save_reserved(): no collection, no erase, one bounded
 * program into cells the last ordinary save left ready. take() is the
 * opposite - it runs on the boot path, where an erase is affordable, so
 * it consumes the record by saving a cleared one through the ORDINARY
 * save(), which also restores the reserve for the next failure.
 */
template <auto& Journal, uint8_t id = 0>
struct JournalPanic {
    JournalPanic() = delete;

    /// The panic Reporter interface (kernel/panic.hpp).
    static void report(PanicCode code, uint8_t context) {
        (void)Journal.save_reserved(
            id, PanicRecord{panic_magic, static_cast<uint8_t>(code), context});
    }

    /// Store a record that was not produced by panic() - the breadcrumb
    /// a boot found in SRAM, on its way somewhere it will still be
    /// tomorrow. Ordinary path: an erase is affordable here.
    static bool save(const PanicRecord& r) {
        return Journal.template save<PanicRecord>(id, r);
    }

    /// Peek without consuming.
    static std::optional<PanicRecord> peek() {
        const std::optional<PanicRecord> r =
            Journal.template load<PanicRecord>(id);
        if (r && r->magic != panic_magic) {
            return std::nullopt;
        }
        return r;
    }

    /// Boot-side fetch-and-clear: a stored record is returned ONCE and
    /// a cleared one is appended in its place. A second call returns
    /// nothing.
    static std::optional<PanicRecord> take() {
        const std::optional<PanicRecord> r = peek();
        if (r) {
            (void)Journal.template save<PanicRecord>(id, PanicRecord{0, 0, 0});
        }
        return r;
    }

    /// True when a record is waiting.
    static bool pending() { return peek().has_value(); }
};

} // namespace brio
