/*
 * nvm_flash.hpp
 *
 * The top of the board's quad-SPI chip as a FlashMedia
 * (util/nv_heap.hpp's contract): `QspiFlashPartition`, where the line is
 * drawn, and `QspiFlash`, the medium itself.
 *
 * A MEDIUM AND NOTHING ABOVE IT. The block heap (util/nv_heap.hpp) and
 * the value journal (util/nv_journal.hpp) stand on this contract on
 * other targets; on this family neither is instantiated, and that is a
 * DECISION and not an omission. What a program here gets is a BAND OF
 * FLASH with bounds it cannot write outside of, and whatever it keeps
 * there is its own structure. The reason is the one CLAUDE.md records as
 * the NV stack's own review: the heap and the journal were born where
 * flash was cheap to partition and carried to every target since for
 * coherence, and the question of WHICH programs need a block allocator,
 * which need small values, and whether a zone at a fixed address is
 * worth a partition every image pays is prior to any port. Until that
 * review, a new family gets the medium and stops there.
 *
 * A CONSTANT PARTITION, DRAWN ONCE, AT THE TOP. `flash` in
 * rp2350/ld/rp2350_16m.ld is 16320K where the chip is 16384K, so the
 * last 64 kB are not the linker's and nothing the compiler emits can
 * land there: the floor is a CONSTANT rather than a symbol that moves
 * with every build, which is what util/nv_heap.hpp's placement rule
 * wants and what keeps an image from growing into its own storage.
 * `__brio_flash_end` is read back as the proof; a script edited past the
 * line CLOSES the medium (`zones()` answers a band whose floor is its
 * ceiling, which every user of the contract refuses on) instead of
 * letting the two overlap.
 *
 *   0x00'0000 .. 0xff'0000   the image (the linker's)
 *   0xff'0000 .. 0x100'0000  QspiFlash, 64 kB: sixteen sectors of 4096
 *
 * THE CELL IS THE PAGE. The bootrom's program function takes whole
 * 256-byte pages (rp2350/flash.hpp), so `write_cell` is 256 under an
 * `erase_size` of 4096: sixteen cells a sector. The chip itself
 * programs a byte at a time and accepts a second program into a page as
 * long as only more bits are cleared, which the suite asks it; a smaller
 * cell would want a program that takes less than a page, which the
 * bootrom's does not.
 *
 * ADDRESSES ARE THE CHIP'S OWN, from 0 - what the bootrom's functions
 * take - and rp2350/flash.hpp adds the window where a pointer is wanted.
 * util/nv_heap.hpp numbers erase units in a uint16_t: 16 MB / 4 kB =
 * 4096, well inside it.
 *
 * A WRITE IS A HOLE IN THE PROGRAM, not a wait. This is the chip the
 * core executes from: for the whole of an erase or a program the memory
 * windows are disconnected, the engine runs from SRAM with this core's
 * interrupts masked, and the kernel loses whatever ticks fall inside
 * (rp2350/flash.hpp's header says what else the application owes -
 * above all that the OTHER CORE must not fetch from flash meanwhile,
 * which nothing here can enforce).
 */

#pragma once

#include <stdint.h>

#include <array>
#include <span>

#include "rp2350/flash.hpp"
#include "util/nv_heap.hpp"

extern "C" {
/// The epoch of the link, injected by rp2350/CMakeLists.txt. Declared as
/// an array so that no code can be tempted to dereference it: it is a
/// VALUE, and only its address bits are ever read.
extern const char __nvheap_build_id[];

/// One past the last byte the LINKER may place, from ld/rp2350_16m.ld:
/// an address in the cached window.
extern const char __brio_flash_end[];
}

namespace brio {

/// Where the line is drawn: one place, read by the medium and by
/// anything that wants to state the geometry.
struct QspiFlashPartition {
    QspiFlashPartition() = delete;

    static constexpr uint32_t sector = Flash::sector_size;                 // 4096
    static constexpr uint32_t page = Flash::page_size;                     // 256
    static constexpr uint32_t storage_sectors = 16;                        // 64 kB
    static constexpr uint32_t storage_end = Flash::size_bytes;             // 0x100'0000 on a 16 MB chip
    static constexpr uint32_t storage_base = storage_end - storage_sectors * sector;

    static_assert(storage_base % sector == 0u);
    static_assert(storage_base > 0u, "brio QspiFlashPartition: the chip is smaller than the zone");
    static_assert(storage_end / sector <= 0xFFFFu,
                  "util/nv_heap.hpp numbers erase units in sixteen bits");

    /// Does the linker script still stop where this partition assumes?
    /// A build whose `flash` region reaches into the zone answers false,
    /// and the medium closes itself.
    static bool geometry_matches_silicon() {
        const auto end = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_flash_end));
        return end >= Flash::window && end - Flash::window <= storage_base;
    }

    /// Is the run [addr, addr + bytes) inside the zone? The COMPILE-TIME
    /// half of the bounds check the medium makes at run time, for a
    /// program that keeps something at an address it knows when it is
    /// built: `static_assert(QspiFlashPartition::contains(a, n))` is how
    /// such an address is pinned.
    static constexpr bool contains(uint32_t addr, uint32_t bytes) {
        return addr >= storage_base && addr <= storage_end && bytes <= storage_end - addr;
    }

    /// Is the run a whole number of CELLS at a cell boundary? The other
    /// half of what `program()` insists on, asked at compile time.
    static constexpr bool cell_aligned(uint32_t addr, uint32_t bytes) {
        return (addr % page) == 0u && (bytes % page) == 0u;
    }
};

/**
 * The zone behind the FlashMedia contract:
 *
 *   static_assert(brio::FlashMedia<brio::QspiFlash>);
 *
 *   uint8_t page[brio::QspiFlash::write_cell];      // in SRAM, always
 *   const auto zone = brio::QspiFlash::zones()[0];
 *   brio::QspiFlash::erase(zone.floor);
 *   brio::QspiFlash::program(zone.floor, page);
 *
 * The bounds check in `program()` and `erase()` is not decoration: a
 * caller that miscounted would otherwise be able to erase the running
 * image, and the bootrom would carry it out without a word.
 */
struct QspiFlash {
    QspiFlash() = delete;

    static constexpr uint32_t erase_size = QspiFlashPartition::sector;   // 4096
    static constexpr uint32_t write_cell = QspiFlashPartition::page;     // 256: whole pages, or nothing
    static constexpr uint32_t flash_end = QspiFlashPartition::storage_end;
    static constexpr uint8_t zone_count = 1;

    static std::array<FlashZone, zone_count> zones() {
        if (!QspiFlashPartition::geometry_matches_silicon()) {
            return {FlashZone{flash_end, flash_end}};
        }
        return {FlashZone{flash_end, QspiFlashPartition::storage_base}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) { Flash::read(addr, dst); }

    /// Whole pages inside the zone and nowhere else.
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        const uint32_t bytes = static_cast<uint32_t>(src.size());
        if (!QspiFlashPartition::contains(addr, bytes) ||
            !QspiFlashPartition::cell_aligned(addr, bytes)) {
            return false;
        }
        return Flash::program(addr, src);
    }

    /// One sector. Same bounds.
    static bool erase(uint32_t addr) {
        if (!QspiFlashPartition::contains(addr, erase_size) || (addr % erase_size) != 0u) {
            return false;
        }
        return Flash::erase_sector(addr);
    }

    static uint32_t build_id() {
        return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&__nvheap_build_id[0]));
    }
};

static_assert(FlashMedia<QspiFlash>);
static_assert(QspiFlash::flash_end % QspiFlash::erase_size == 0u);
static_assert(QspiFlash::flash_end / QspiFlash::erase_size <= 0xFFFFu);

} // namespace brio
