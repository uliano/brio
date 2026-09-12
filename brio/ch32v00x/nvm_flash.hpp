/*
 * nvm_flash.hpp
 *
 * The CH32V00x's main flash as a FlashMedia (util/nv_heap.hpp): the
 * backends that let the block allocator and the value journal run on
 * this silicon, where the storage and the running image share ONE
 * array of 62 KB.
 *
 * A CONSTANT PARTITION, drawn once. The linker script gives the linker
 * the first 40 KB only (`rom` in ch32v00x/ld/ch32v006k8.ld), so
 * nothing the compiler emits can land above 0xA000, and the storage's
 * floor and homes are constants rather than symbols that move with
 * every build - the STM32G0's arrangement, for the same reasons:
 * util/nv_heap.hpp's placement rule wants a floor that stays put, and
 * an image that grew into its own storage would be a silent loss. The
 * script's own `__brio_rom_end` is read back as the proof; a script
 * edited past 0xA000 closes the storage (both users read a zone with
 * its floor at its ceiling as "refuse") instead of letting an image
 * grow into it.
 *
 *   0x0000 .. 0xA000   the image (40 KB, the linker's)
 *   0xA000 .. 0xE000   MainFlash             16 KB, 64 pages  blocks
 *                                            (util/nv_heap.hpp), its map
 *                                            pair in the top two pages
 *   0xE000 .. 0xF800   MainFlashJournalZone   6 KB, 24 pages  small values
 *                                            (util/nv_journal.hpp), two
 *                                            halves of twelve pages
 *
 * THE CELL IS THE PAGE, and that is the fact that sizes everything
 * here. This family programs by whole 256-byte pages and nothing
 * smaller (nvm.hpp), so `write_cell` is 256 and coincides with
 * `erase_size`. For the journal every entry therefore costs a page:
 * a half of twelve pages holds twelve entries, and its geometry
 * assertion, (max_ids + 2) x max_entry_cells <= half_cells, allows up
 * to ten ids - a handful of small values is what the journal is for,
 * and the attic is sized for a handful. For the heap an append of any
 * length costs a page too. What a smaller cell would need is a page
 * that accepts a SECOND program between erases with more bits cleared;
 * the chapter does not promise it, and the storage suite has a letter
 * that asks the silicon.
 *
 * ADDRESSES ARE THE ARRAY'S OWN, from 0 - the addresses the image
 * runs at - and the engine adds the 0x0800 0000 alias where the
 * chapter wants it. util/nv_heap.hpp numbers erase units in a
 * uint16_t: 0xF800 / 256 = 248, no problem here.
 *
 * NO READ-WHILE-WRITE: this array is the one the core executes from,
 * and a page program or erase stalls the fetch for its duration (a
 * few milliseconds for an erase). An ordinary NvJournal::save() from
 * the main loop is therefore a pause of the whole program, not a
 * background operation as it is in the STM32G0's second bank or the
 * SAM C21's RWWEE array - a fact for the application, measured by the
 * storage suite.
 */

#pragma once

#include <stdint.h>

#include <array>
#include <span>

#include "ch32v00x/nvm.hpp"
#include "util/nv_heap.hpp"

extern "C" {
/// The epoch of the link, injected by ch32v00x/CMakeLists.txt. Declared
/// as an array so that no code can be tempted to dereference it: it is
/// a VALUE, and only its address bits are ever read.
extern const char __nvheap_build_id[];

/// One past the last byte the LINKER may place, from ld/<part>.ld.
extern const char __brio_rom_end[];
}

namespace brio {

/// Where the lines are drawn: one place, read by both media and by
/// anything that wants to state the geometry.
struct MainFlashPartition {
    MainFlashPartition() = delete;

    static constexpr uint32_t page = Flash::page_size;                 // 256 on the CH32V006, 64 on the CH32V003
    static constexpr uint32_t storage_base = device::flash_program_bytes;   // 0xA000 on the CH32V006, 0x3C00 on the CH32V003
    static constexpr uint32_t storage_end = Flash::array_bytes;        // 0xF800 / 0x4000
    static constexpr uint32_t journal_pages = device::flash_journal_pages;
    static constexpr uint32_t journal_bytes = journal_pages * page;    // 6 KB / 1 KB
    static constexpr uint32_t journal_base = storage_end - journal_bytes;   // 0xE000 / 0x3C00
    static constexpr uint32_t heap_end = journal_base;                 // the heap's share: 16 KB on the CH32V006, none on the CH32V003

    static_assert(storage_base % page == 0u && journal_base % page == 0u);
    static_assert(storage_base <= journal_base && journal_base < storage_end);
    static_assert((storage_base < journal_base) == device::has_flash_heap);

    /// Does the linker script still stop where this partition assumes?
    static bool geometry_matches_silicon() {
        return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_rom_end)) <=
               storage_base;
    }
};

namespace detail {

/// The mechanics both backends share: bounds, the lock window. Not a
/// FlashMedia itself - no zones, no flash_end - so nothing can mount it
/// by accident.
struct MainFlashOps {
    MainFlashOps() = delete;

    static void read(uint32_t addr, std::span<uint8_t> dst) {
        Flash::read(addr, dst);
    }

    /**
     * Program whole pages inside [floor, ceiling) and nowhere else.
     * The bounds check is not decoration: a heap that miscounted would
     * otherwise be able to program the running image, and the silicon
     * would carry it out. THE UNLOCK WINDOW IS THIS CALL: both locks
     * open for the pages and shut again on the way out.
     */
    static bool program(uint32_t addr, std::span<const uint8_t> src,
                        uint32_t floor, uint32_t ceiling) {
        // `addr >= ceiling` first: without it `ceiling - addr` wraps.
        if (addr < floor || addr >= ceiling || src.size() > ceiling - addr ||
            (addr % Flash::page_size) != 0u || (src.size() % Flash::page_size) != 0u) {
            return false;
        }
        if (!Flash::unlock()) {
            return false;
        }
        uint32_t err = 0;
        for (uint32_t off = 0; off < src.size() && err == 0u; off += Flash::page_size) {
            err = Flash::program_page(addr + off, src.subspan(off, Flash::page_size));
        }
        Flash::lock();
        return err == 0u;
    }

    /// One page. Same bounds, same window.
    static bool erase(uint32_t addr, uint32_t floor, uint32_t ceiling) {
        if (addr < floor || addr >= ceiling || (addr % Flash::page_size) != 0u) {
            return false;
        }
        if (!Flash::unlock()) {
            return false;
        }
        const uint32_t err = Flash::erase_page(addr);
        Flash::lock();
        return err == 0u;
    }

    static uint32_t build_id() {
        return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&__nvheap_build_id[0]));
    }

    /// What zones() answers when the linker script no longer stops at
    /// the partition's floor: a band with its floor at its ceiling,
    /// which both util users refuse on.
    static constexpr FlashZone refusal(uint32_t end) { return FlashZone{end, end}; }
};

} // namespace detail

#if BRIO_CH32_HAS_FLASH_HEAP
/// The heap's share behind the FlashMedia contract.
struct MainFlash {
    MainFlash() = delete;

    static constexpr uint32_t erase_size = Flash::page_size;   // 256
    static constexpr uint32_t write_cell = Flash::page_size;   // 256: the page IS the cell
    static constexpr uint32_t flash_end = MainFlashPartition::heap_end;
    static constexpr uint8_t zone_count = 1;

    static std::array<FlashZone, zone_count> zones() {
        if (!MainFlashPartition::geometry_matches_silicon()) {
            return {detail::MainFlashOps::refusal(flash_end)};
        }
        return {FlashZone{flash_end, MainFlashPartition::storage_base}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) {
        detail::MainFlashOps::read(addr, dst);
    }
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        return detail::MainFlashOps::program(addr, src, MainFlashPartition::storage_base,
                                             flash_end);
    }
    static bool erase(uint32_t addr) {
        return detail::MainFlashOps::erase(addr, MainFlashPartition::storage_base, flash_end);
    }
    static uint32_t build_id() { return detail::MainFlashOps::build_id(); }
};

static_assert(FlashMedia<MainFlash>);
static_assert(MainFlash::flash_end % MainFlash::erase_size == 0u);
static_assert(MainFlash::flash_end / MainFlash::erase_size <= 0xFFFFu);
#endif   // the CH32V003's array has no heap share (device::has_flash_heap)

/// The journal's share: the attic, the top 24 pages.
struct MainFlashJournalZone {
    MainFlashJournalZone() = delete;

    static constexpr uint32_t erase_size = Flash::page_size;   // 256
    static constexpr uint32_t write_cell = Flash::page_size;   // 256
    static constexpr uint32_t flash_end = MainFlashPartition::storage_end;
    static constexpr uint8_t zone_count = 1;

    static std::array<FlashZone, zone_count> zones() {
        if (!MainFlashPartition::geometry_matches_silicon()) {
            return {detail::MainFlashOps::refusal(flash_end)};
        }
        return {FlashZone{flash_end, MainFlashPartition::journal_base}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) {
        detail::MainFlashOps::read(addr, dst);
    }
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        return detail::MainFlashOps::program(addr, src, MainFlashPartition::journal_base,
                                             flash_end);
    }
    static bool erase(uint32_t addr) {
        return detail::MainFlashOps::erase(addr, MainFlashPartition::journal_base,
                                           flash_end);
    }
    static uint32_t build_id() { return detail::MainFlashOps::build_id(); }
};

static_assert(FlashMedia<MainFlashJournalZone>);
static_assert(MainFlashJournalZone::flash_end % MainFlashJournalZone::erase_size == 0u);

} // namespace brio
