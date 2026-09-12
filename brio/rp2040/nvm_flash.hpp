/*
 * nvm_flash.hpp
 *
 * The RP2040's flash as a FlashMedia (util/nv_heap.hpp): the backends
 * that let the block allocator and the value journal run on this
 * silicon, where the storage and the running image share ONE external
 * quad-SPI chip that the core executes out of.
 *
 * A CONSTANT PARTITION, drawn once at the top of the chip. The linker
 * script gives the linker everything but the last 64 KB (`flash` in
 * rp2040/ld/rp2040_2m.ld stops at 1984K), so nothing the compiler
 * emits can land there, and the storage's floor and homes are
 * constants rather than symbols that move with every build - the
 * STM32G0's and the CH32V00x's arrangement, for the same reasons:
 * util/nv_heap.hpp's placement rule wants a floor that stays put, and
 * an image that grew into its own storage would be a silent loss. The
 * script's own `__brio_flash_end` is read back as the proof; a script
 * edited past the floor closes the storage (both users read a zone
 * with its floor at its ceiling as "refuse") instead of letting an
 * image grow into it. On a 2 MB chip:
 *
 *   0x000000 .. 0x1F0000   the stage and the image (the linker's)
 *   0x1F0000 .. 0x1FC000   QspiFlash              48 KB, 12 sectors  blocks
 *                                                 (util/nv_heap.hpp), its map
 *                                                 pair in the top two sectors
 *   0x1FC000 .. 0x200000   QspiFlashJournalZone   16 KB, 4 sectors   small values
 *                                                 (util/nv_journal.hpp), two
 *                                                 halves of two sectors
 *
 * THE CELL IS THE PAGE. The bootrom's program takes whole 256-byte
 * pages (flash.hpp), so `write_cell` is 256 under an `erase_size` of
 * 4096: sixteen cells a sector, a journal half of two sectors holding
 * thirty-two entries, and the heap's map entries and headers in cells
 * of 256. The chip itself programs a byte at a time and accepts a
 * second program into a page as long as only more bits are cleared
 * (the suite asks it); a smaller cell would want a program that takes
 * less than a page, which the bootrom's does not.
 *
 * ADDRESSES ARE THE CHIP'S OWN, from 0 - what the bootrom's functions
 * take - and the engine adds the window where a pointer is wanted.
 * util/nv_heap.hpp numbers erase units in a uint16_t: 2 MB / 4 KB =
 * 512, no problem here.
 *
 * NO READ-WHILE-WRITE, AND MORE: this chip is the one the core executes
 * from, and for the whole of an erase or a program the flash is
 * DISCONNECTED - the engine runs from SRAM with interrupts masked
 * (flash.hpp). An ordinary NvJournal::save() from the main loop is a
 * pause of the whole program - under a millisecond for the page, tens
 * of milliseconds when it erases a sector - during which no interrupt
 * is served and the ticker loses its ticks; the suite measures both.
 * The other core and the DMA must keep off the window for that time,
 * which nothing here enforces.
 */

#pragma once

#include <stdint.h>

#include <array>
#include <span>

#include "rp2040/flash.hpp"
#include "util/nv_heap.hpp"

extern "C" {
/// The epoch of the link, injected by rp2040/CMakeLists.txt. Declared
/// as an array so that no code can be tempted to dereference it: it is
/// a VALUE, and only its address bits are ever read.
extern const char __nvheap_build_id[];

/// One past the last byte the LINKER may place, from ld/rp2040_2m.ld:
/// an address in the window.
extern const char __brio_flash_end[];
}

namespace brio {

/// Where the lines are drawn: one place, read by both media and by
/// anything that wants to state the geometry.
struct QspiFlashPartition {
    QspiFlashPartition() = delete;

    static constexpr uint32_t sector = Flash::sector_size;             // 4096
    static constexpr uint32_t page = Flash::page_size;                 // 256
    static constexpr uint32_t storage_sectors = 16;                    // 64 KB
    static constexpr uint32_t journal_sectors = 4;                     // 16 KB, two halves of two
    static constexpr uint32_t storage_end = Flash::size_bytes;         // 0x200000 on a 2 MB chip
    static constexpr uint32_t storage_base = storage_end - storage_sectors * sector;   // 0x1F0000
    static constexpr uint32_t journal_base = storage_end - journal_sectors * sector;   // 0x1FC000
    static constexpr uint32_t heap_end = journal_base;                 // the heap's share: 48 KB
    static constexpr uint32_t heap_sectors = storage_sectors - journal_sectors;

    static_assert(storage_base % sector == 0u && journal_base % sector == 0u);
    static_assert(storage_base < journal_base && journal_base < storage_end);
    static_assert(storage_end / sector <= 0xFFFFu, "util/nv_heap.hpp numbers erase units in sixteen bits");

    /// Does the linker script still stop where this partition assumes?
    static bool geometry_matches_silicon() {
        const auto end = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_flash_end));
        return end >= Flash::window && end - Flash::window <= storage_base;
    }
};

namespace detail {

/// The mechanics both backends share: the bounds. Not a FlashMedia
/// itself - no zones, no flash_end - so nothing can mount it by
/// accident.
struct QspiFlashOps {
    QspiFlashOps() = delete;

    static void read(uint32_t addr, std::span<uint8_t> dst) { Flash::read(addr, dst); }

    /**
     * Program whole pages inside [floor, ceiling) and nowhere else.
     * The bounds check is not decoration: a heap that miscounted would
     * otherwise be able to program the running image, and the bootrom
     * would carry it out.
     */
    static bool program(uint32_t addr, std::span<const uint8_t> src, uint32_t floor, uint32_t ceiling) {
        // `addr >= ceiling` first: without it `ceiling - addr` wraps.
        if (addr < floor || addr >= ceiling || src.size() > ceiling - addr) {
            return false;
        }
        return Flash::program(addr, src);
    }

    /// One sector. Same bounds.
    static bool erase(uint32_t addr, uint32_t floor, uint32_t ceiling) {
        if (addr < floor || addr >= ceiling) {
            return false;
        }
        return Flash::erase_sector(addr);
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

/// The heap's share behind the FlashMedia contract.
struct QspiFlash {
    QspiFlash() = delete;

    static constexpr uint32_t erase_size = QspiFlashPartition::sector;   // 4096
    static constexpr uint32_t write_cell = QspiFlashPartition::page;     // 256: the bootrom programs whole pages
    static constexpr uint32_t flash_end = QspiFlashPartition::heap_end;
    static constexpr uint8_t zone_count = 1;

    static std::array<FlashZone, zone_count> zones() {
        if (!QspiFlashPartition::geometry_matches_silicon()) {
            return {detail::QspiFlashOps::refusal(flash_end)};
        }
        return {FlashZone{flash_end, QspiFlashPartition::storage_base}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) { detail::QspiFlashOps::read(addr, dst); }
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        return detail::QspiFlashOps::program(addr, src, QspiFlashPartition::storage_base, flash_end);
    }
    static bool erase(uint32_t addr) {
        return detail::QspiFlashOps::erase(addr, QspiFlashPartition::storage_base, flash_end);
    }
    static uint32_t build_id() { return detail::QspiFlashOps::build_id(); }
};

static_assert(FlashMedia<QspiFlash>);
static_assert(QspiFlash::flash_end % QspiFlash::erase_size == 0u);
static_assert(QspiFlash::flash_end / QspiFlash::erase_size <= 0xFFFFu);

/// The journal's share: the attic, the top four sectors.
struct QspiFlashJournalZone {
    QspiFlashJournalZone() = delete;

    static constexpr uint32_t erase_size = QspiFlashPartition::sector;   // 4096
    static constexpr uint32_t write_cell = QspiFlashPartition::page;     // 256
    static constexpr uint32_t flash_end = QspiFlashPartition::storage_end;
    static constexpr uint8_t zone_count = 1;

    static std::array<FlashZone, zone_count> zones() {
        if (!QspiFlashPartition::geometry_matches_silicon()) {
            return {detail::QspiFlashOps::refusal(flash_end)};
        }
        return {FlashZone{flash_end, QspiFlashPartition::journal_base}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) { detail::QspiFlashOps::read(addr, dst); }
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        return detail::QspiFlashOps::program(addr, src, QspiFlashPartition::journal_base, flash_end);
    }
    static bool erase(uint32_t addr) {
        return detail::QspiFlashOps::erase(addr, QspiFlashPartition::journal_base, flash_end);
    }
    static uint32_t build_id() { return detail::QspiFlashOps::build_id(); }
};

static_assert(FlashMedia<QspiFlashJournalZone>);
static_assert(QspiFlashJournalZone::flash_end % QspiFlashJournalZone::erase_size == 0u);

} // namespace brio
