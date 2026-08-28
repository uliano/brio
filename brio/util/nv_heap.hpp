/*
 * nv_heap.hpp
 *
 * NvHeap: runtime-allocated blocks of flash that survive a reboot - and,
 * on a bench that reflashes page-selectively, a reflash.
 *
 * WHAT IT IS FOR. The EEPROM (util/nv_record.hpp) is the right home for
 * a handful of small values rewritten often: byte-granular, 100k cycles.
 * It is the wrong home for a calibration table, a font, a captured
 * waveform - kilobytes, written once in a while, read constantly. That
 * belongs in the flash the image does not occupy, and the only thing
 * standing between an application and that flash is bookkeeping: which
 * pages are free, where a block starts, how long it is, and whether what
 * is there is still the thing that was written.
 *
 * THE MAP PAIR. All the bookkeeping lives in ONE structure, held in the
 * last `map_pages` erase units of the part - an address anchored to the
 * silicon, not to the linker, so no image can be built over it by
 * accident. A map version is one whole-unit write: header, up to
 * `max_blocks` entries, CRC. Versions ping-pong: every mutation erases
 * the OLDEST of the rotation pages and writes the next version there
 * with seq + 1, and the current map is the highest-seq version whose
 * magic and CRC verify. Atomicity is therefore structural, not
 * defensive - the old version rules the heap until the new one is fully
 * down, and a version torn by a power loss simply fails its CRC. (The
 * same discipline is what keeps an ECC-guarded flash happy: no cell is
 * ever programmed twice between erases.)
 *
 * BLOCKS ARE PURE PAYLOAD. No headers, no chaining, no in-band markers:
 * a block is `size_pages` erase units of application bytes and nothing
 * else, so find() hands out an address and a length and the reader is
 * free to walk it however it likes. Everything that describes it -
 * record id, length, checksum, flags - is in the map entry.
 *
 * SURVIVAL-AWARE MOUNT, AND WHY THE BUILD ID IS ONLY A DIAGNOSTIC.
 * mount() is READ-ONLY: it costs no erase cycle at boot. It picks the
 * current version, then verifies EVERY listed block by reading its
 * payload and checking the recorded CRC. What verifies is served by
 * find(); what does not is reported as lost and disappears from the next
 * published version - its pages are free again the moment the map knows
 * it. The application therefore learns at boot exactly which of its
 * tables came through and which it must recreate. Validity is judged by
 * the CHECKSUM, never by which build wrote the block: the build id
 * travels in the map because it is worth seeing, not because anything
 * decides on it. A table whose MEANING changes between firmware
 * versions handles that in its own record id or in its own payload
 * versioning - the heap cannot know it.
 *
 * PLACEMENT. The free flash is described by the media as ZONES, each a
 * ceiling and a floor, both of which are runtime facts (on a real target
 * they come from linker symbols). A new block goes as HIGH as it fits:
 * in each zone the topmost gap among the live blocks that accommodates
 * it, and then the zone where the placed block ends up FARTHEST from its
 * own floor. No growth model is assumed - the rule measures the free
 * space that is actually there, which is the only honest thing to do
 * when whether the code or the read-only data grows next is a property
 * of the application and not of the allocator.
 *
 * THE MAP-HOME GUARD. The linker knows nothing about the map pages, so
 * nothing stops an image from growing its read-only data up into them.
 * mount() refuses - with a distinct status and no write of any kind - if
 * a zone floor has climbed into the map home, because publishing a map
 * version would then erase a page of the running image's own constants.
 * A refused mount serves nothing and mutates nothing.
 *
 * WHAT IS DELIBERATELY NOT HERE. There is no free() and there are no
 * tombstones: a block is superseded by allocating its id again (the old
 * one stays current until the new one is sealed, so a power loss in
 * between loses nothing), and a block whose payload no longer verifies
 * frees itself. There is no wear levelling of the payload pages - a
 * block is written where it was placed, and flash that is rewritten
 * often does not belong in flash at all (that is what the EEPROM is
 * for). There is no journaling or ring log: a growing accumulator is a
 * different structure with different invariants and it waits for a real
 * user.
 *
 * WEAR. Reads and mount() cost nothing. Each mutation costs one erase of
 * one map page plus the erases of the block being written; the rotation
 * spreads the map's own wear over `map_pages` pages, so the map endures
 * map_pages x (the part's per-page cycle budget) mutations.
 */

#pragma once

#include <stdint.h>

#include <concepts>
#include <optional>
#include <span>

#include "util/crc.hpp"

namespace brio {

/**
 * A band of flash the heap may use, as the media reports it: `floor` is
 * the first usable byte, `ceiling` the byte one past the last. Both are
 * runtime values - on a real target they are linker symbols rounded to
 * the erase granularity, and they move with every build.
 */
struct FlashZone {
    uint32_t ceiling;
    uint32_t floor;

    constexpr bool empty() const { return ceiling <= floor; }
    constexpr uint32_t size() const { return empty() ? 0u : ceiling - floor; }
};

/**
 * The flash behind an NvHeap. Everything here is static: a memory is a
 * piece of hardware, not an object.
 *
 *  - erase_size   bytes in one erase unit. The allocation granularity:
 *                 a block occupies whole erase units, because a
 *                 neighbour's erase would otherwise take it down.
 *  - write_cell   bytes in one program unit, written ONCE between
 *                 erases (a word here, an ECC-guarded double-word
 *                 there). Kept separate from erase_size on purpose:
 *                 "page" means different things on different silicon.
 *  - flash_end    byte address one past the last flash byte. The map
 *                 home is anchored to it, so it must be a compile-time
 *                 constant and a multiple of erase_size.
 *  - zone_count   how many usable bands the media reports.
 *  - zones()      the bands themselves, indexable, `zone_count` of them.
 *                 A RUNTIME call: the floors come from the image being
 *                 built and are not knowable when the heap is compiled.
 *  - read(addr, dst)      fill dst from flash. Never fails: a read of
 *                         erased flash returns the erased pattern.
 *  - program(addr, src)   program src at addr. Address and size are
 *                         multiples of write_cell and the target cells
 *                         have been erased since they were last
 *                         programmed. False = nothing (or not all of it)
 *                         went down.
 *  - erase(addr)          erase the one unit at addr (a multiple of
 *                         erase_size).
 *  - build_id()           an identifier of the running image, recorded
 *                         in every map version. Diagnostic only - no
 *                         decision in this header consults it.
 */
template <typename M>
concept FlashMedia = requires(uint32_t addr, uint8_t index,
                              std::span<uint8_t> dst,
                              std::span<const uint8_t> src) {
    { M::erase_size } -> std::convertible_to<uint32_t>;
    { M::write_cell } -> std::convertible_to<uint32_t>;
    { M::flash_end } -> std::convertible_to<uint32_t>;
    { M::zone_count } -> std::convertible_to<uint8_t>;
    { M::zones()[index] } -> std::convertible_to<FlashZone>;
    { M::read(addr, dst) };
    { M::program(addr, src) } -> std::convertible_to<bool>;
    { M::erase(addr) } -> std::convertible_to<bool>;
    { M::build_id() } -> std::convertible_to<uint32_t>;
};

/// What mount() found.
enum class NvHeapStatus : uint8_t {
    unmounted = 0,  ///< mount() has not run yet
    ok,             ///< a current map version was read (losses are in the report)
    empty,          ///< no version verified: virgin flash, or all of it unreadable
    bad_geometry,   ///< the media's zones are malformed, or a floor has climbed
                    ///< into the map home - REFUSED, nothing was written
};

/**
 * One block, as the map describes it. Public because an application that
 * wants to walk its own heap (a status console, a diagnostic dump) has
 * every right to; nothing here is written by hand.
 */
struct NvBlockEntry {
    uint16_t record_id;   ///< the application's name for the block
    uint16_t first_page;  ///< erase unit index from the base of the flash
    uint16_t size_pages;  ///< erase units reserved, >= 1
    uint32_t payload_len; ///< bytes actually written and covered by the CRC
    uint16_t payload_crc; ///< CRC-16 of those bytes
    uint8_t flags;        ///< reserved for policy (nothing reads it yet)
    uint8_t reserved;
};

/**
 * A block found by NvHeap::find(): where it is, how long it is, and a
 * bounds-checked read. Reading is a media call and costs nothing else -
 * a block is plain flash, and an application that prefers to walk it
 * with its own target-specific reader is welcome to the address.
 */
template <FlashMedia Media>
struct NvBlock {
    uint16_t record_id;
    uint32_t address;
    uint32_t length;

    bool read(uint32_t offset, std::span<uint8_t> dst) const {
        if (offset > length || dst.size() > length - offset) {
            return false;
        }
        Media::read(address + offset, dst);
        return true;
    }
};

/**
 * The flash block allocator. One instance per media; the constructor is
 * trivial and constexpr (RAM only, no global constructor, no flash
 * touched), and NOTHING happens until mount() is called - after the
 * clock is up, where a failure can be reported.
 *
 *   max_blocks  how many blocks may be live at once. It sizes the RAM
 *               index and the map entry table, and is static_asserted
 *               against what one erase unit can physically hold.
 *   map_pages   erase units of the rotation. Two is the minimum a
 *               ping-pong needs and the default; more only buys wear.
 */
template <FlashMedia Media, uint8_t max_blocks = 8, uint8_t map_pages = 2>
class NvHeap {
public:
    static constexpr uint32_t erase_size = Media::erase_size;
    static constexpr uint32_t write_cell = Media::write_cell;
    static constexpr uint32_t flash_end = Media::flash_end;

    /// The map's home: the last `map_pages` erase units of the part.
    static constexpr uint32_t map_home =
        flash_end - static_cast<uint32_t>(map_pages) * erase_size;

    /// The map version's on-flash layout, little-endian throughout:
    ///   0   magic u32        4   format u8      5   entry_count u8
    ///   6   reserved u16     8   seq u32       12   build_id u32
    ///   16  entry[max_blocks], 14 bytes each:
    ///         0 record_id u16   2 first_page u16   4 size_pages u16
    ///         6 payload_len u32 10 payload_crc u16 12 flags u8
    ///        13 reserved u8
    ///   then crc16 u16 over every preceding byte.
    static constexpr uint32_t magic = 0x5048564Eu;   // "NVHP", little-endian
    static constexpr uint8_t format_version = 1;
    static constexpr uint16_t header_bytes = 16;
    static constexpr uint16_t entry_bytes = 14;
    static constexpr uint16_t map_bytes =
        static_cast<uint16_t>(header_bytes + entry_bytes * max_blocks + 2);
    /// What is actually programmed: whole write cells, the tail padded.
    static constexpr uint16_t map_image_bytes =
        static_cast<uint16_t>((map_bytes + write_cell - 1u) / write_cell *
                              write_cell);

    static_assert(max_blocks >= 1, "a heap with no blocks is not a heap");
    static_assert(map_pages >= 2,
                  "the map is a ping-pong: with one page a mutation would "
                  "erase the only copy and the heap would not be crash-safe");
    static_assert(erase_size >= write_cell &&
                      erase_size % write_cell == 0,
                  "an erase unit must be a whole number of program units");
    static_assert(flash_end % erase_size == 0,
                  "the flash must end on an erase-unit boundary: the map home "
                  "is anchored to that address");
    static_assert(map_image_bytes <= erase_size,
                  "max_blocks is larger than one erase unit of this media can "
                  "hold: a map version is ONE unit-sized write");
    static_assert(static_cast<uint32_t>(map_pages) * erase_size < flash_end,
                  "the map home would swallow the whole part");

    /// What mount() found, in full: the status, the current version's own
    /// numbers, and the record ids on each side of the CRC verdict.
    struct MountReport {
        NvHeapStatus status = NvHeapStatus::unmounted;
        uint8_t survivors = 0;
        uint8_t lost = 0;
        uint32_t seq = 0;       ///< the current version's sequence number
        uint32_t build_id = 0;  ///< the image that wrote it (diagnostic)
        uint16_t survivor_ids[max_blocks]{};
        uint16_t lost_ids[max_blocks]{};

        bool mounted() const {
            return status == NvHeapStatus::ok || status == NvHeapStatus::empty;
        }
    };

    class Writer;

    constexpr NvHeap() = default;

    NvHeap(const NvHeap&) = delete;
    NvHeap& operator=(const NvHeap&) = delete;

    // ---- mounting ----------------------------------------------------------

    /**
     * Read the map, verify every block, and open the heap for business.
     * Costs no erase cycle: this is the boot path.
     *
     * Returns the report, which is also available afterwards from
     * report(). A bad_geometry verdict leaves the heap CLOSED - alloc(),
     * rewrite() and find() all refuse - because the only thing a heap
     * whose map home is occupied by the running image can do is damage.
     */
    const MountReport& mount() {
        report_ = MountReport{};
        count_ = 0;
        seq_ = 0;
        current_page_ = 0;
        mounted_ = false;
        for (uint8_t i = 0; i < map_pages; ++i) {
            page_seq_[i] = 0;
        }
        for (uint8_t i = 0; i < max_blocks; ++i) {
            pending_used_[i] = false;
        }

        if (!geometry_ok()) {
            report_.status = NvHeapStatus::bad_geometry;
            return report_;
        }

        bool found = false;
        for (uint8_t p = 0; p < map_pages; ++p) {
            uint8_t image[map_bytes];
            Media::read(map_home + static_cast<uint32_t>(p) * erase_size,
                        std::span<uint8_t>(image, map_bytes));
            if (!version_valid(image)) {
                continue;
            }
            const uint32_t seq = load32(image + 8);
            page_seq_[p] = seq;
            if (found && seq <= seq_) {
                continue;
            }
            found = true;
            seq_ = seq;
            current_page_ = p;
            report_.build_id = load32(image + 12);
            count_ = image[5];
            for (uint8_t i = 0; i < count_; ++i) {
                index_[i] = load_entry(image + header_bytes +
                                       static_cast<uint16_t>(i) * entry_bytes);
            }
        }

        // Every listed block is judged by its own checksum, and only by
        // it: what verifies survives, what does not is reported lost and
        // is gone from the next version published.
        uint8_t live = 0;
        for (uint8_t i = 0; i < count_; ++i) {
            const NvBlockEntry& e = index_[i];
            if (entry_sane(e) && payload_ok(e)) {
                if (live != i) {
                    index_[live] = e;
                }
                report_.survivor_ids[report_.survivors++] = e.record_id;
                ++live;
            } else {
                report_.lost_ids[report_.lost++] = e.record_id;
            }
        }
        count_ = live;
        mounted_ = true;
        report_.status = found ? NvHeapStatus::ok : NvHeapStatus::empty;
        report_.seq = seq_;
        return report_;
    }

    const MountReport& report() const { return report_; }
    bool mounted() const { return mounted_; }
    /// The current map version's sequence number (0 = none published yet).
    uint32_t sequence() const { return seq_; }
    /// Which erase unit of the rotation holds the current version.
    uint8_t map_page() const { return current_page_; }

    /// The live blocks, in the order the map lists them.
    uint8_t count() const { return count_; }
    const NvBlockEntry& entry(uint8_t i) const { return index_[i]; }

    // ---- reading -----------------------------------------------------------

    /// The live block with this record id, if the heap holds one.
    std::optional<NvBlock<Media>> find(uint16_t id) const {
        if (!mounted_) {
            return std::nullopt;
        }
        for (uint8_t i = 0; i < count_; ++i) {
            if (index_[i].record_id == id) {
                return NvBlock<Media>{id, address_of(index_[i]),
                                      index_[i].payload_len};
            }
        }
        return std::nullopt;
    }

    // ---- writing -----------------------------------------------------------

    /**
     * Reserve room for `len` bytes under `id` and erase it, ready to be
     * filled. Nothing about the heap changes until the returned handle
     * is sealed: a block with the same id stays live and findable, and a
     * power loss before the seal leaves the heap exactly as it was.
     *
     * Refused (nothing written) when the heap is not mounted, when len
     * is 0, when no gap in any zone fits, when the block table is full
     * (live blocks plus unsealed handles), or when the erase fails.
     */
    std::optional<Writer> alloc(uint16_t id, uint32_t len) {
        if (!mounted_ || len == 0) {
            return std::nullopt;
        }
        const uint32_t pages = (len + erase_size - 1u) / erase_size;
        if (pages > 0xFFFFu) {
            return std::nullopt;
        }
        const uint16_t need = static_cast<uint16_t>(pages);
        // A new id needs a new place in the block table; superseding one
        // that is already live does not - the seal REPLACES its entry, so
        // the published version never holds both.
        if (!live(id) &&
            static_cast<uint16_t>(count_) + pending_count() + 1u > max_blocks) {
            return std::nullopt;
        }
        uint16_t first = 0;
        if (!place(need, first)) {
            return std::nullopt;
        }
        const uint8_t slot = free_slot();
        if (slot == max_blocks) {
            return std::nullopt;
        }
        pending_[slot] = NvBlockEntry{id, first, need, 0, 0xFFFFu, 0, 0};
        pending_used_[slot] = true;
        if (!erase_block(first, need)) {
            pending_used_[slot] = false;
            return std::nullopt;
        }
        return std::optional<Writer>(Writer(this, slot));
    }

    /**
     * Refill a live block IN PLACE: same address, same reserved size,
     * new contents. The payload pages are erased at once, so from this
     * moment until the seal the block's recorded checksum no longer
     * describes what is on the flash - a power loss here is a LOSS, and
     * the next mount() reports it as one. That is the honest cost of
     * rewriting a block without a second copy of it; an application that
     * cannot afford it allocates the id twice and alternates.
     */
    std::optional<Writer> rewrite(uint16_t id) {
        if (!mounted_) {
            return std::nullopt;
        }
        for (uint8_t i = 0; i < count_; ++i) {
            if (index_[i].record_id != id) {
                continue;
            }
            const uint8_t slot = free_slot();
            if (slot == max_blocks) {
                return std::nullopt;
            }
            pending_[slot] = NvBlockEntry{id, index_[i].first_page,
                                          index_[i].size_pages, 0, 0xFFFFu,
                                          index_[i].flags, 0};
            pending_used_[slot] = true;
            if (!erase_block(index_[i].first_page, index_[i].size_pages)) {
                pending_used_[slot] = false;
                return std::nullopt;
            }
            return std::optional<Writer>(Writer(this, slot));
        }
        return std::nullopt;
    }

    /**
     * The handle an alloc() or rewrite() hands out: an append-only
     * stream into one block, and the seal that publishes it.
     *
     * Bytes are buffered until a whole program unit is ready, so the
     * caller may append in any chunking it likes - one byte or a
     * kilobyte - without knowing what the media's program unit is. The
     * running checksum is computed over exactly the bytes appended; the
     * padding of the last unit is not part of the payload.
     *
     * A handle destroyed without sealing abandons the reservation: the
     * pages it erased stay free, the map is untouched, and any block of
     * the same id that was live before is still live.
     */
    class Writer {
    public:
        Writer(const Writer&) = delete;
        Writer& operator=(const Writer&) = delete;
        Writer& operator=(Writer&&) = delete;

        Writer(Writer&& other) noexcept
            : heap_(other.heap_), slot_(other.slot_), written_(other.written_),
              crc_(other.crc_), fill_(other.fill_), failed_(other.failed_),
              done_(other.done_) {
            for (uint32_t i = 0; i < write_cell; ++i) {
                cell_[i] = other.cell_[i];
            }
            other.heap_ = nullptr;
        }

        ~Writer() { abandon(); }

        /// The block's first byte in the flash.
        uint32_t address() const {
            return heap_ != nullptr ? heap_->address_of(heap_->pending_[slot_])
                                    : 0u;
        }
        /// The bytes the reservation can hold: whole erase units.
        uint32_t capacity() const {
            return heap_ != nullptr
                       ? static_cast<uint32_t>(heap_->pending_[slot_].size_pages) *
                             erase_size
                       : 0u;
        }
        uint32_t written() const { return written_; }
        uint16_t record_id() const {
            return heap_ != nullptr ? heap_->pending_[slot_].record_id : 0u;
        }
        /// True once a program has failed or the payload has overflowed:
        /// the handle is dead and seal() will refuse.
        bool failed() const { return failed_; }
        bool sealed() const { return done_; }

        /// Append bytes to the block. False when the handle is spent,
        /// when the payload would overrun the reservation, or when the
        /// media refused a program - and in the last two cases the
        /// handle is failed for good.
        bool append(std::span<const uint8_t> data) {
            if (heap_ == nullptr || failed_ || done_) {
                return false;
            }
            if (data.size() > capacity() - written_) {
                failed_ = true;
                return false;
            }
            for (const uint8_t b : data) {
                cell_[fill_++] = b;
                crc_ = crc16_byte(crc_, b);
                ++written_;
                if (fill_ == write_cell) {
                    fill_ = 0;
                    if (!heap_->program_cell(address() + written_ - write_cell,
                                             cell_)) {
                        failed_ = true;
                        return false;
                    }
                }
            }
            return true;
        }

        /// Flush the partial program unit (padded with the erased
        /// pattern) and PUBLISH: a new map version listing this block
        /// and every survivor except a previous block of the same id.
        /// This is the commit point - before it, nothing about the heap
        /// has changed; after it, the new block is the current one.
        bool seal() {
            if (heap_ == nullptr || failed_ || done_) {
                return false;
            }
            if (fill_ != 0) {
                const uint32_t at = address() + written_ - fill_;
                for (uint32_t i = fill_; i < write_cell; ++i) {
                    cell_[i] = 0xFFu;
                }
                fill_ = 0;
                if (!heap_->program_cell(at, cell_)) {
                    failed_ = true;
                    return false;
                }
            }
            NvBlockEntry e = heap_->pending_[slot_];
            e.payload_len = written_;
            e.payload_crc = crc_;
            if (!heap_->commit(e)) {
                failed_ = true;
                return false;
            }
            done_ = true;
            heap_->pending_used_[slot_] = false;
            heap_ = nullptr;
            return true;
        }

    private:
        friend class NvHeap;

        Writer(NvHeap* heap, uint8_t slot) : heap_(heap), slot_(slot) {}

        void abandon() {
            if (heap_ != nullptr) {
                heap_->pending_used_[slot_] = false;
                heap_ = nullptr;
            }
        }

        NvHeap* heap_ = nullptr;
        uint8_t slot_ = 0;
        uint32_t written_ = 0;
        uint16_t crc_ = 0xFFFFu;
        uint32_t fill_ = 0;
        bool failed_ = false;
        bool done_ = false;
        uint8_t cell_[write_cell]{};
    };

    // ---- what the free space looks like right now ---------------------------

    /// The zones as the heap uses them: the media's, rounded inwards to
    /// whole erase units and with the map home carved out.
    FlashZone zone(uint8_t i) const {
        const FlashZone z = Media::zones()[i];
        uint32_t ceiling = z.ceiling / erase_size * erase_size;
        if (ceiling > map_home) {
            ceiling = map_home;
        }
        const uint32_t floor = (z.floor + erase_size - 1u) / erase_size * erase_size;
        return FlashZone{ceiling < floor ? floor : ceiling, floor};
    }

    /// Erase units of this zone not occupied by a live block or an
    /// unsealed reservation. They need not be contiguous.
    uint16_t free_pages(uint8_t i) const {
        const FlashZone z = zone(i);
        uint16_t free = static_cast<uint16_t>(z.size() / erase_size);
        for (uint8_t k = 0; k < count_; ++k) {
            free = static_cast<uint16_t>(free - overlap(index_[k], z));
        }
        for (uint8_t k = 0; k < max_blocks; ++k) {
            if (pending_used_[k]) {
                free = static_cast<uint16_t>(free - overlap(pending_[k], z));
            }
        }
        return free;
    }

private:
    friend class Writer;

    // ---- geometry ----------------------------------------------------------

    /// The zones must be inside the part, right way up, and - the guard
    /// this whole check exists for - none of them may have a FLOOR above
    /// the map home. A floor is the first byte the heap may use, so
    /// everything below it belongs to the running image: a floor inside
    /// the map home means the image's own bytes are sitting where a map
    /// version would be written, and publishing one would erase them.
    bool geometry_ok() const {
        for (uint8_t i = 0; i < Media::zone_count; ++i) {
            const FlashZone z = Media::zones()[i];
            if (z.ceiling > flash_end || z.floor > z.ceiling) {
                return false;
            }
            if (z.floor > map_home) {
                return false;
            }
        }
        return true;
    }

    static uint32_t address_of(const NvBlockEntry& e) {
        return static_cast<uint32_t>(e.first_page) * erase_size;
    }

    /// A map version can carry an entry that no longer describes
    /// anything reachable (an older image with a different geometry, a
    /// media that shrank). Refuse it before reading through it.
    static bool entry_sane(const NvBlockEntry& e) {
        if (e.size_pages == 0) {
            return false;
        }
        const uint32_t begin = address_of(e);
        const uint32_t end =
            begin + static_cast<uint32_t>(e.size_pages) * erase_size;
        if (end > map_home || end <= begin) {
            return false;
        }
        return e.payload_len <= end - begin;
    }

    /// Pages of `e` that fall inside `z`.
    static uint16_t overlap(const NvBlockEntry& e, const FlashZone& z) {
        const uint32_t begin = address_of(e);
        const uint32_t end =
            begin + static_cast<uint32_t>(e.size_pages) * erase_size;
        const uint32_t lo = begin > z.floor ? begin : z.floor;
        const uint32_t hi = end < z.ceiling ? end : z.ceiling;
        return hi > lo ? static_cast<uint16_t>((hi - lo) / erase_size) : 0u;
    }

    // ---- placement ---------------------------------------------------------

    /// The lowest start page among the reservations overlapping
    /// [lo, hi), or `hi` when the window is clear.
    uint16_t blocker(uint16_t lo, uint16_t hi) const {
        uint16_t lowest = hi;
        for (uint8_t i = 0; i < count_; ++i) {
            lowest = blocker_of(index_[i], lo, hi, lowest);
        }
        for (uint8_t i = 0; i < max_blocks; ++i) {
            if (pending_used_[i]) {
                lowest = blocker_of(pending_[i], lo, hi, lowest);
            }
        }
        return lowest;
    }

    static uint16_t blocker_of(const NvBlockEntry& e, uint16_t lo, uint16_t hi,
                               uint16_t lowest) {
        const uint16_t begin = e.first_page;
        const uint16_t end = static_cast<uint16_t>(e.first_page + e.size_pages);
        if (begin < hi && end > lo && begin < lowest) {
            return begin;
        }
        return lowest;
    }

    /// The TOPMOST window of `need` free pages in this zone.
    bool topmost_fit(const FlashZone& z, uint16_t need, uint16_t& page) const {
        if (z.empty()) {
            return false;
        }
        const uint16_t floor_p = static_cast<uint16_t>(z.floor / erase_size);
        uint16_t top = static_cast<uint16_t>(z.ceiling / erase_size);
        for (;;) {
            if (top < floor_p || top - floor_p < need) {
                return false;
            }
            const uint16_t lo = static_cast<uint16_t>(top - need);
            const uint16_t b = blocker(lo, top);
            if (b == top) {
                page = lo;
                return true;
            }
            top = b;   // strictly below the window: the search descends
        }
    }

    /// Top-down in each zone, then the zone that leaves the block
    /// farthest above its own floor. Measured clearance, no growth model.
    bool place(uint16_t need, uint16_t& page) const {
        bool found = false;
        uint32_t best_clearance = 0;
        for (uint8_t i = 0; i < Media::zone_count; ++i) {
            const FlashZone z = zone(i);
            uint16_t candidate = 0;
            if (!topmost_fit(z, need, candidate)) {
                continue;
            }
            const uint32_t clearance =
                static_cast<uint32_t>(candidate) * erase_size - z.floor;
            if (!found || clearance > best_clearance) {
                found = true;
                best_clearance = clearance;
                page = candidate;
            }
        }
        return found;
    }

    // ---- the media, through one door each -----------------------------------

    bool erase_block(uint16_t first, uint16_t pages) {
        for (uint16_t i = 0; i < pages; ++i) {
            if (!Media::erase((static_cast<uint32_t>(first) + i) * erase_size)) {
                return false;
            }
        }
        return true;
    }

    bool program_cell(uint32_t addr, const uint8_t* cell) {
        return Media::program(addr, std::span<const uint8_t>(cell, write_cell));
    }

    bool payload_ok(const NvBlockEntry& e) const {
        uint32_t addr = address_of(e);
        uint32_t left = e.payload_len;
        uint16_t crc = 0xFFFFu;
        uint8_t buf[32];
        while (left != 0) {
            const uint16_t chunk =
                static_cast<uint16_t>(left < sizeof buf ? left : sizeof buf);
            Media::read(addr, std::span<uint8_t>(buf, chunk));
            for (uint16_t i = 0; i < chunk; ++i) {
                crc = crc16_byte(crc, buf[i]);
            }
            addr += chunk;
            left -= chunk;
        }
        return crc == e.payload_crc;
    }

    // ---- the map ------------------------------------------------------------

    uint8_t pending_count() const {
        uint8_t n = 0;
        for (uint8_t i = 0; i < max_blocks; ++i) {
            n = static_cast<uint8_t>(n + (pending_used_[i] ? 1 : 0));
        }
        return n;
    }

    bool live(uint16_t id) const {
        for (uint8_t i = 0; i < count_; ++i) {
            if (index_[i].record_id == id) {
                return true;
            }
        }
        return false;
    }

    uint8_t free_slot() const {
        for (uint8_t i = 0; i < max_blocks; ++i) {
            if (!pending_used_[i]) {
                return i;
            }
        }
        return max_blocks;
    }

    /// Publish the survivors, minus any older block of this id, plus this
    /// one. The heap's RAM index is updated only if the version lands.
    bool commit(const NvBlockEntry& e) {
        NvBlockEntry next[max_blocks];
        uint8_t n = 0;
        for (uint8_t i = 0; i < count_; ++i) {
            if (index_[i].record_id == e.record_id) {
                continue;
            }
            if (n == max_blocks) {
                return false;
            }
            next[n++] = index_[i];
        }
        if (n == max_blocks) {
            return false;
        }
        next[n++] = e;
        if (!publish(next, n)) {
            return false;
        }
        for (uint8_t i = 0; i < n; ++i) {
            index_[i] = next[i];
        }
        count_ = n;
        return true;
    }

    /// One whole-unit write into the oldest page of the rotation. The
    /// current version is untouched until this one is entirely down, and
    /// a torn write simply fails its CRC at the next mount.
    bool publish(const NvBlockEntry* entries, uint8_t n) {
        uint8_t image[map_image_bytes];
        for (uint16_t i = 0; i < map_image_bytes; ++i) {
            image[i] = 0xFFu;
        }
        store32(image, magic);
        image[4] = format_version;
        image[5] = n;
        image[6] = 0;
        image[7] = 0;
        const uint32_t seq = seq_ + 1u;
        store32(image + 8, seq);
        store32(image + 12, Media::build_id());
        for (uint8_t i = 0; i < max_blocks; ++i) {
            uint8_t* const at =
                image + header_bytes + static_cast<uint16_t>(i) * entry_bytes;
            store_entry(at, i < n ? entries[i] : NvBlockEntry{});
        }
        const uint16_t crc = crc16(image, static_cast<uint16_t>(map_bytes - 2));
        store16(image + map_bytes - 2, crc);

        const uint8_t page = oldest_page();
        const uint32_t addr = map_home + static_cast<uint32_t>(page) * erase_size;
        if (!Media::erase(addr)) {
            return false;
        }
        page_seq_[page] = 0;
        if (!Media::program(addr,
                            std::span<const uint8_t>(image, map_image_bytes))) {
            return false;
        }
        seq_ = seq;
        current_page_ = page;
        page_seq_[page] = seq;
        return true;
    }

    /// The rotation page holding the oldest version (an unreadable one
    /// counts as the oldest of all) - never the current one, since two
    /// pages is the enforced minimum.
    uint8_t oldest_page() const {
        uint8_t best = 0;
        for (uint8_t i = 1; i < map_pages; ++i) {
            if (page_seq_[i] < page_seq_[best]) {
                best = i;
            }
        }
        return best;
    }

    static bool version_valid(const uint8_t* image) {
        if (load32(image) != magic || image[4] != format_version ||
            image[5] > max_blocks) {
            return false;
        }
        return crc16(image, static_cast<uint16_t>(map_bytes - 2)) ==
               load16(image + map_bytes - 2);
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

    static void store_entry(uint8_t* at, const NvBlockEntry& e) {
        store16(at, e.record_id);
        store16(at + 2, e.first_page);
        store16(at + 4, e.size_pages);
        store32(at + 6, e.payload_len);
        store16(at + 10, e.payload_crc);
        at[12] = e.flags;
        at[13] = e.reserved;
    }
    static NvBlockEntry load_entry(const uint8_t* at) {
        return NvBlockEntry{load16(at),      load16(at + 2), load16(at + 4),
                            load32(at + 6),  load16(at + 10), at[12],
                            at[13]};
    }

    // ---- state --------------------------------------------------------------

    MountReport report_{};
    NvBlockEntry index_[max_blocks]{};
    NvBlockEntry pending_[max_blocks]{};
    bool pending_used_[max_blocks]{};
    uint32_t page_seq_[map_pages]{};
    uint32_t seq_ = 0;
    uint8_t count_ = 0;
    uint8_t current_page_ = 0;
    bool mounted_ = false;
};

} // namespace brio
