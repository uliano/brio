/*
 * nvm_flash.hpp
 *
 * The CH32V203's main flash as a FlashMedia (util/nv_heap.hpp's
 * contract): `MainFlashPartition`, where the line is drawn, and
 * `MainFlash<Clock>`, the medium itself.
 *
 * A MEDIUM AND NOTHING ABOVE IT. The heap (util/nv_heap.hpp) and the
 * journal (util/nv_journal.hpp) stand on this contract on other
 * targets; on this family neither is instantiated, and that is a
 * decision rather than an omission - what a program here gets is a
 * BAND OF FLASH with bounds it cannot write outside of, and whatever it
 * keeps there is its own structure. Two facts stand behind the
 * decision, and the second of them is this silicon's own:
 *
 *  - THE ERASED PATTERN IS NOT 0xFF. An erased cell of this array reads
 *    back 0xE339E339 (RM 32.5.4 and 32.5.7, said four times), where
 *    every other flash this repository writes leaves all ones. The heap
 *    judges a map version by its magic and its CRC and would not care;
 *    the journal finds the end of a half by looking for a CELL OF 0xFF
 *    BYTES, so it would have to be taught this pattern before it could
 *    stand here at all.
 *  - THE CELL IS THE PAGE, 256 bytes, which is a coarse grain for the
 *    journal's small values: one entry would cost a page. The chapter
 *    offers a HALF-WORD program beside the page one (nvm.hpp's
 *    `program_half_word`), and the bench has since measured that a cell
 *    written that way TAKES PASS AFTER PASS between erases, each
 *    reading back exactly what was written - so a two-byte cell is
 *    possible on this silicon, and the page is the cell here by the
 *    decision above and not for want of a mechanism. What such a store
 *    would still have to be taught is the erased pattern, and that a
 *    half-word program costs more than the fast method spends on a
 *    whole page (2.6 ms against 1.5).
 *
 * A CONSTANT PARTITION, DRAWN ONCE, AT THE TOP. The zone is the LAST
 * 4 KB of the array on every part of the series - sixteen pages of 256
 * bytes, and exactly one WRITE-PROTECTION unit (RM 32.1's note 1), so
 * the storage can be protected or left open without touching the
 * image's own sectors. Every linker script of this project stops its
 * flash region 4 KB short of the array (ch32vx03/ld/<part>.ld), so
 * nothing the compiler emits can land there and the floor is a
 * CONSTANT rather than a symbol that moves with every build - the same
 * arrangement the sister family uses, and for the same reason:
 * util/nv_heap.hpp's placement rule wants a floor that stays put, and
 * an image that grew into its own storage would be a silent loss.
 * `__brio_rom_end` is read back as the proof; a script edited past the
 * line CLOSES the medium (`zones()` answers a band whose floor is its
 * ceiling, which every user of the contract refuses on) instead of
 * letting the two overlap.
 *
 *   0x0000 .. array - 4K   the image (the linker's)
 *   array - 4K .. array    MainFlash, 16 pages of 256 bytes
 *
 * ADDRESSES ARE THE ARRAY'S OWN, counted from zero - the addresses the
 * image runs at - and nvm.hpp adds the 0x0800 0000 alias where the
 * chapter wants it. util/nv_heap.hpp numbers erase units in a uint16_t:
 * the largest part of this series is 128 KB, 512 pages, no problem.
 *
 * THE CLOCK IS PART OF THE MEDIUM'S TYPE, and this is the one place
 * where this stratum's FlashMedia differs in shape from the others'.
 * An erase or a program on this silicon is legal only below
 * `flash_safe_sysclk_hz` (nvm.hpp's header says why), and the contract's
 * `program()` and `erase()` take an address and nothing else - so the
 * rate has to arrive with the TYPE. `MainFlash<Clk>` is therefore
 * written with the program's own clock, which makes the refusal a
 * compile error under a static one and a `false` under a dynamic one,
 * at the same place the contract already allows a false.
 *
 * A WRITE IS A WAIT AND NOT A STALL, which is the one thing about this
 * medium that the other families here would have predicted wrongly.
 * This is the array the core executes from, and yet MEASURED on the
 * CH32V203C8 the core goes on running out of it while the engine works:
 * across a page erase the ruler showed 9.8 ms and the kernel's
 * millisecond tick - which counts HANDLER RUNS, so it can only advance
 * if the core could fetch the handler - advanced by the same nine or
 * ten. A page program costs 1.5 ms the same way. So an erase or a
 * program here is a
 * BLOCKING CALL for its caller (the driver polls BSY) and not a hole in
 * the program: interrupts keep being served throughout.
 */

#pragma once

#include <stdint.h>

#include <array>
#include <span>

#include "ch32vx03/nvm.hpp"
#include "util/nv_heap.hpp"

extern "C" {
/// The epoch of the link, injected by ch32vx03/CMakeLists.txt. Declared
/// as an array so that no code can be tempted to dereference it: it is
/// a VALUE, and only its address bits are ever read.
extern const char __nvheap_build_id[];

/// One past the last byte the LINKER may place, from ld/<part>.ld.
extern const char __brio_rom_end[];
}

namespace brio {

/// Where the line is drawn: one place, read by the medium and by
/// anything that wants to state the geometry.
struct MainFlashPartition {
    MainFlashPartition() = delete;

    static constexpr uint32_t page = Flash::page_size;            // 256
    /// The zone is one write-protection unit, which is also the
    /// standard erase grain: 4 KB, sixteen pages.
    static constexpr uint32_t zone_bytes = Flash::sector_size;    // 4096
    static constexpr uint32_t zone_pages = zone_bytes / page;     // 16
    static constexpr uint32_t storage_end = Flash::array_bytes;   // the top of the array
    static constexpr uint32_t storage_base = storage_end - zone_bytes;

    static_assert(storage_base % page == 0u);
    static_assert(zone_bytes % page == 0u);
    static_assert(storage_base > 0u, "brio MainFlashPartition: the array is smaller than the zone");

    /// Does the linker script still stop where this partition assumes?
    /// A build whose `rom` region reaches into the zone answers false,
    /// and the medium closes itself.
    static bool geometry_matches_silicon() {
        return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__brio_rom_end)) <= storage_base;
    }

    /// Is the run [addr, addr + bytes) inside the zone? The
    /// COMPILE-TIME half of the bounds check the medium makes at run
    /// time, for a program that keeps something at an address it knows
    /// when it is built: `static_assert(MainFlashPartition::contains(a,
    /// n))` is how such an address is pinned.
    static constexpr bool contains(uint32_t addr, uint32_t bytes) {
        return addr >= storage_base && addr <= storage_end && bytes <= storage_end - addr;
    }

    /// Is the run a whole number of CELLS at a cell boundary? The other
    /// half of what `program()` will insist on, asked at compile time.
    static constexpr bool cell_aligned(uint32_t addr, uint32_t bytes) {
        return (addr % page) == 0u && (bytes % page) == 0u;
    }
};

/**
 * The zone behind the FlashMedia contract, written with the clock the
 * program runs on:
 *
 *   using Clk = brio::Clock<brio::ClockSource::pll, 96'000'000>;
 *   using Store = brio::MainFlash<Clk>;
 *   static_assert(brio::FlashMedia<Store>);
 *
 *   uint8_t page[Store::write_cell];
 *   Store::erase(Store::zones()[0].floor);
 *   Store::program(Store::zones()[0].floor, page);
 *
 * THE UNLOCK WINDOW IS ONE CALL: both of the engine's locks are opened
 * for the pages of one `program()` or `erase()` and shut again on the
 * way out, so an operation that fails leaves the engine closed.
 */
template <typename Clock>
struct MainFlash {
    MainFlash() = delete;

    static constexpr uint32_t erase_size = Flash::page_size;   // 256
    static constexpr uint32_t write_cell = Flash::page_size;   // 256: the page IS the cell
    static constexpr uint32_t flash_end = MainFlashPartition::storage_end;
    static constexpr uint8_t zone_count = 1;

    /// What a read of a never-written cell answers here, published
    /// because it is NOT the all-ones every other medium leaves (the
    /// file header).
    static constexpr uint32_t erased_word = Flash::erased_word;

    static std::array<FlashZone, zone_count> zones() {
        if (!MainFlashPartition::geometry_matches_silicon()) {
            return {FlashZone{flash_end, flash_end}};
        }
        return {FlashZone{flash_end, MainFlashPartition::storage_base}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) { Flash::read(addr, dst); }

    /**
     * Program whole pages inside the zone and nowhere else. The bounds
     * check is not decoration: a caller that miscounted would otherwise
     * be able to program the running image, and the silicon would carry
     * it out.
     */
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        const uint32_t bytes = static_cast<uint32_t>(src.size());
        if (!MainFlashPartition::contains(addr, bytes) ||
            !MainFlashPartition::cell_aligned(addr, bytes)) {
            return false;
        }
        if (!Flash::unlock()) {
            return false;
        }
        uint32_t err = 0;
        for (uint32_t off = 0; off < bytes && err == 0u; off += write_cell) {
            err = Flash::program_page(Clock{}, addr + off, src.subspan(off, write_cell));
        }
        Flash::lock();
        return err == 0u;
    }

    /// One page. Same bounds, same window.
    static bool erase(uint32_t addr) {
        if (!MainFlashPartition::contains(addr, erase_size) ||
            !MainFlashPartition::cell_aligned(addr, erase_size)) {
            return false;
        }
        if (!Flash::unlock()) {
            return false;
        }
        const uint32_t err = Flash::erase_page(Clock{}, addr);
        Flash::lock();
        return err == 0u;
    }

    /// Diagnostic only: no decision in util/nv_heap.hpp consults it.
    static uint32_t build_id() {
        return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&__nvheap_build_id[0]));
    }
};

static_assert(MainFlashPartition::storage_end % Flash::page_size == 0u);
static_assert(MainFlashPartition::storage_end / Flash::page_size <= 0xFFFFu);
static_assert(MainFlashPartition::contains(MainFlashPartition::storage_base,
                                           MainFlashPartition::zone_bytes));
static_assert(!MainFlashPartition::contains(MainFlashPartition::storage_base -
                                                MainFlashPartition::page,
                                            MainFlashPartition::page));

} // namespace brio
