/*
 * nvm_flash.hpp
 *
 * The STM32G0's main flash as a FlashMedia (util/nv_heap.hpp): the
 * backends that let the target-independent block allocator and the
 * target-independent value journal run on this silicon - the THIRD
 * implementation of that contract, and the first one on a part where the
 * storage and the running image share one array.
 *
 * THE STORAGE ATTIC IS BANK 2, and that choice is the whole design.
 *
 * This part has 512 Kbytes of flash in TWO BANKS of 256 (RM0444 table
 * 11), and three of the chapter's facts point the same way:
 *
 *  - READ-WHILE-WRITE (3.3.9): programming or erasing one bank leaves
 *    reads of the OTHER bank running. Inside one bank a read stalls the
 *    bus until the operation ends (3.3.6) - which, for code executing
 *    from that bank, means the CPU stops for the whole 22..40 ms of a
 *    page erase. So a program that stores a record while it runs must
 *    store it in the bank it does not execute from. That is what makes
 *    an ordinary NvJournal::save() legal from the main loop here, the
 *    same property the samc backend bought by living in the RWWEE array.
 *
 *  - ERRATUM ES0548 2.2.10 (LIVE, no workaround): the prefetch may fail
 *    when the CPU BRANCHES ACROSS BANKS. The erratum's own note then
 *    lists what stays safe, first item: "EEPROM emulation or other data
 *    storage in bank 2". Code must not straddle the banks; data in the
 *    far bank is the vendor's own sanctioned use of the feature.
 *
 *  - A CONSTANT ZONE. stm32g0/../ld/stm32g0b1re.ld gives the linker BANK
 *    1 ONLY (256 K of rom), so nothing the compiler emits can land in
 *    bank 2. The heap's floor and the journal's home are therefore
 *    CONSTANTS, exactly as they are on the samc's RWWEE array and unlike
 *    the AVR's, where the free flash is bounded by linker symbols that
 *    move with every build. The linker script's own `__brio_rom_end` is
 *    read back here as the proof, so a script edited back to 512 K
 *    closes the storage instead of letting an image grow into it.
 *
 * ADDRESSES HERE ARE OFFSETS FROM 0x0800 0000, not absolute, and that is
 * a decision with a reason. util/nv_heap.hpp numbers erase units in a
 * uint16_t (NvBlockEntry::first_page), so a media whose addresses are
 * absolute must sit below page 65536 - which the samc's RWWEE array at
 * 0x0040 0000 does (page 16384) and this part's bank 2 at 0x0804 0000
 * does NOT: 0x08040000 / 2048 is 65664, one page past the field. The
 * AVR backend already numbers its flash from zero, so this is the
 * contract's other established convention rather than a new one, and it
 * costs one addition per access.
 *
 * THE ARRAY IS PARTITIONED, because this target has the same two storage
 * classes the samc does and one array to put them in:
 *
 *   pages 0..125 of bank 2   MainFlash             252 K  blocks
 *                                                         (util/nv_heap.hpp),
 *                                                         its map pair in
 *                                                         pages 124..125
 *   pages 126..127           MainFlashJournalZone     4 K  small values
 *                                                         (util/nv_journal.hpp),
 *                                                         two 2 K halves
 *
 * The journal's two halves are ONE PAGE each, which is the smallest a
 * half may be (erasing one must not take the other down) and is already
 * 256 write cells - the geometry assertion in util/nv_journal.hpp,
 * (max_ids + 2) x max_entry_cells <= half_cells, is then satisfied by
 * anything up to 40 ids of 32-byte payloads (a 32-byte value costs
 * ceil((12 + 32) / 8) = 6 cells). Both bounds are anchored to the TOP of
 * the part so each user's home is anchored to the silicon in turn.
 *
 * THE GRANULARITIES, and they are the widest split of the three targets:
 * an erase takes down a PAGE of 2048 bytes and a program writes a DOUBLE
 * WORD of 8. On the samc it is 256 and 64, on the AVR 512 and 2. Code
 * that says "page" for both is code that is wrong on all three.
 *
 * ERRATUM ES0548 2.2.3 IS UNREACHABLE BY CONSTRUCTION here, and it is
 * worth saying which construction. 3.3.8 makes one exception to the
 * write-once rule - a location may be overwritten if the value is all
 * zeros - and the erratum says that exception does not work on this
 * silicon when the location holds all ones. Neither util/nv_heap.hpp nor
 * util/nv_journal.hpp ever programs a cell twice between erases, so
 * neither can ever ask for the exception; program() below does not
 * special-case zeros either, and a caller that tries gets the silicon's
 * PROGERR back unedited.
 *
 * THE BUILD ID is a link-time constant, as on the other two targets:
 * stm32g0/CMakeLists.txt passes -Wl,--defsym,__nvheap_build_id=<newest
 * source mtime> to every image, so an unchanged tree relinks to the same
 * bytes. A pointer is 32 bits here, so the symbol's ADDRESS is the value.
 * It is recorded in every map version and NOTHING decides on it - a
 * block's validity is its checksum's business.
 */

#pragma once

#include <stdint.h>

#include <array>
#include <span>

#include "stm32g0/flash.hpp"
#include "util/nv_heap.hpp"

extern "C" {
/// The epoch of the link, injected by stm32g0/CMakeLists.txt. Declared
/// as an array so that no code can be tempted to dereference it: it is a
/// VALUE, and only its address bits are ever read.
extern const char __nvheap_build_id[];

/// One past the last byte the LINKER may place, from ld/<part>.ld. Read
/// as a value the same way: the storage below is only safe while this
/// stops at the bank boundary.
extern const char __brio_rom_end[];
}

namespace brio {

/**
 * Where the lines are drawn. One place, read by both media below and by
 * anything that wants to state the geometry (a suite, a console) without
 * repeating the arithmetic.
 */
struct MainFlashPartition {
    MainFlashPartition() = delete;

    /// Offsets, not addresses - see the file header.
    static constexpr uint32_t bank_bytes = 256u * 1024u;
    static constexpr uint32_t storage_base = bank_bytes;          // 0x40000
    static constexpr uint32_t storage_end = 2u * bank_bytes;      // 0x80000

    /// Pages of the top given to the journal: two, one per half.
    static constexpr uint32_t journal_pages = 2;
    static constexpr uint32_t journal_bytes = journal_pages * Flash::page_size;
    static constexpr uint32_t journal_base = storage_end - journal_bytes;
    static constexpr uint32_t heap_end = journal_base;

    static_assert(journal_base > storage_base,
                  "the journal would swallow the bank");
    static_assert(storage_base % Flash::page_size == 0 &&
                      journal_base % Flash::page_size == 0,
                  "both regions must start on an erase unit");

    /// A media offset turned back into the address the CPU and the FLASH
    /// engine speak.
    static constexpr uint32_t address_of(uint32_t offset) {
        return Flash::base + offset;
    }

    /**
     * Whether this silicon and this link really are what the partition
     * is drawn for: 512 Kbytes in two banks, with everything the linker
     * placed stopping at the bank boundary.
     *
     * A RUNTIME question, and it has to be. The device-select define is
     * STM32G0B1xx for the 128, 256 and 512 Kbyte members alike, so the
     * flash size is only knowable from the size register; and whether
     * the image was linked into one bank is only knowable from the
     * linker's own symbol. When the answer is no, zones() below reports
     * a region no user can fit in and both mounts refuse with
     * bad_geometry, having written nothing - which is the one safe
     * answer when the map cannot be trusted.
     */
    static bool geometry_matches_silicon() {
        return flash_size_kb() == 512u && Flash::bank_count() == 2u &&
               static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_rom_end)) <=
                   address_of(storage_base);
    }
};

namespace detail {

/// The mechanics both backends share: bounds, the unlock window, the
/// translation to absolute addresses. Not a FlashMedia itself - it has
/// no zones and no flash_end - so nothing can mount it by accident.
struct MainFlashOps {
    MainFlashOps() = delete;

    static void read(uint32_t offset, std::span<uint8_t> dst) {
        Flash::read(MainFlashPartition::address_of(offset), dst);
    }

    /**
     * Program whole double words, inside the storage bank and nowhere
     * else. The bounds check is not decoration: a heap that miscounted
     * would otherwise be able to program the running image, and the
     * silicon would carry it out.
     *
     * THE UNLOCK WINDOW IS THIS CALL. FLASH_CR is unlocked here and
     * locked again on the way out, so the register spends the program's
     * own microseconds writable and the rest of the program's life shut
     * - the cheapest form of the discipline the chapter asks for, and
     * the one that makes a wild store into FLASH_CR a no-op rather than
     * an erase.
     */
    static bool program(uint32_t offset, std::span<const uint8_t> src,
                        uint32_t floor, uint32_t ceiling) {
        // THE `offset >= ceiling` TEST IS NOT REDUNDANT: without it
        // `ceiling - offset` wraps and a wild address past the top of
        // the region passes the size check with room to spare. Every
        // bound here is unsigned, and this is the one place where that
        // matters.
        if (offset < floor || offset >= ceiling ||
            src.size() > ceiling - offset) {
            return false;
        }
        if (!Flash::unlock()) {
            return false;
        }
        const uint32_t err =
            Flash::program(MainFlashPartition::address_of(offset), src);
        (void)Flash::lock();
        return err == 0u;
    }

    /// One page. Same bounds, same window.
    static bool erase(uint32_t offset, uint32_t floor, uint32_t ceiling) {
        if (offset < floor || offset >= ceiling ||
            (offset % Flash::page_size) != 0u) {
            return false;
        }
        if (!Flash::unlock()) {
            return false;
        }
        const uint32_t err =
            Flash::erase_page(MainFlashPartition::address_of(offset));
        (void)Flash::lock();
        return err == 0u;
    }

    static uint32_t build_id() {
        return static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(&__nvheap_build_id[0]));
    }

    /// What zones() answers when the silicon is not the one the
    /// partition was drawn for: a band with its FLOOR at the very top,
    /// which both util users read as "your home is below my floor" and
    /// refuse on (NvHeap::geometry_ok, NvJournal::geometry_ok).
    static constexpr FlashZone refusal(uint32_t end) {
        return FlashZone{end, end};
    }
};

} // namespace detail

/**
 * The heap's share of bank 2 behind the FlashMedia contract. All static:
 * there is one flash, and it is a piece of hardware.
 */
struct MainFlash {
    MainFlash() = delete;

    static constexpr uint32_t erase_size = Flash::page_size;   // 2048
    static constexpr uint32_t write_cell = Flash::cell_size;   // 8
    static constexpr uint32_t flash_end = MainFlashPartition::heap_end;
    static constexpr uint8_t zone_count = 1;

    /// Pages 0..125 of bank 2. The heap carves its own map home out of
    /// the top of this by itself, which with the default two map pages
    /// is pages 124..125.
    static std::array<FlashZone, zone_count> zones() {
        if (!MainFlashPartition::geometry_matches_silicon()) {
            return std::array<FlashZone, zone_count>{
                detail::MainFlashOps::refusal(flash_end)};
        }
        return std::array<FlashZone, zone_count>{
            FlashZone{flash_end, MainFlashPartition::storage_base}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) {
        detail::MainFlashOps::read(addr, dst);
    }
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        return detail::MainFlashOps::program(addr, src,
                                             MainFlashPartition::storage_base,
                                             flash_end);
    }
    static bool erase(uint32_t addr) {
        return detail::MainFlashOps::erase(addr, MainFlashPartition::storage_base,
                                           flash_end);
    }
    static uint32_t build_id() { return detail::MainFlashOps::build_id(); }
};

static_assert(FlashMedia<MainFlash>);
static_assert(MainFlash::flash_end % MainFlash::erase_size == 0u,
              "the map home is anchored to flash_end");
static_assert(MainFlash::flash_end / MainFlash::erase_size <= 0xFFFFu,
              "NvHeap numbers erase units in a uint16_t - which is why this "
              "media's addresses are offsets from 0x08000000 and not the "
              "absolute addresses the samc backend can afford");

/**
 * The journal's share: the attic, the top two pages of bank 2.
 *
 * A second FlashMedia rather than a parameter on the first one, because
 * the contract is a whole MEMORY and both users anchor their own home to
 * their media's flash_end. Two media over one bank is what keeps that
 * true for both without either knowing the other exists - the samc
 * partition's shape, one page size up.
 */
struct MainFlashJournalZone {
    MainFlashJournalZone() = delete;

    static constexpr uint32_t erase_size = Flash::page_size;   // 2048
    static constexpr uint32_t write_cell = Flash::cell_size;   // 8
    static constexpr uint32_t flash_end = MainFlashPartition::storage_end;
    static constexpr uint8_t zone_count = 1;

    /// The attic and nothing else. A journal with one-page halves fills
    /// it exactly, so its own geometry check is satisfied on the
    /// boundary.
    static std::array<FlashZone, zone_count> zones() {
        if (!MainFlashPartition::geometry_matches_silicon()) {
            return std::array<FlashZone, zone_count>{
                detail::MainFlashOps::refusal(flash_end)};
        }
        return std::array<FlashZone, zone_count>{
            FlashZone{flash_end, MainFlashPartition::journal_base}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) {
        detail::MainFlashOps::read(addr, dst);
    }
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        return detail::MainFlashOps::program(addr, src,
                                             MainFlashPartition::journal_base,
                                             flash_end);
    }
    static bool erase(uint32_t addr) {
        return detail::MainFlashOps::erase(addr, MainFlashPartition::journal_base,
                                           flash_end);
    }
    static uint32_t build_id() { return detail::MainFlashOps::build_id(); }
};

static_assert(FlashMedia<MainFlashJournalZone>);
static_assert(MainFlashJournalZone::flash_end % MainFlashJournalZone::erase_size == 0u,
              "the journal's halves are anchored to flash_end");
static_assert(MainFlashJournalZone::flash_end / MainFlashJournalZone::erase_size <=
                  0xFFFFu,
              "the same uint16_t page-number ceiling");

} // namespace brio
