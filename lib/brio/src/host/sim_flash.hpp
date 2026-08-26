/*
 * sim_flash.hpp
 *
 * A FlashMedia (util/nv_heap.hpp) made of RAM, with the three things a
 * real one cannot offer: an arbitrary GEOMETRY, a COUNTER on every kind
 * of wear, and a POWER SWITCH that can be thrown between any two program
 * units.
 *
 * WHY THE GEOMETRY IS A PARAMETER. "Page" means 512 bytes erased and two
 * bytes programmed on an AVR Dx, and 2 KB erased and eight ECC-guarded
 * bytes programmed on an STM32G0. A block allocator that is only ever
 * exercised against one of those shapes is an allocator with the other
 * shape's bugs still in it, so the host suite runs the same source over
 * both.
 *
 * WHAT IT MODELS OF REAL FLASH. Programming only clears bits (the stored
 * byte is ANDed with the new one), an erase puts a whole unit back to
 * 0xFF, and every program unit remembers whether it has already been
 * programmed since its last erase - the discipline ECC memories require,
 * and one that no amount of testing on a forgiving part would reveal.
 * Violations are COUNTED, not refused: a test asserts the count is zero,
 * which is a stronger statement than "the media said no".
 *
 * WHAT IT MODELS OF A POWER LOSS. cut_after(n) lets exactly n more
 * program units land and then takes the power away: the (n+1)-th program
 * writes nothing, returns false, and every later program and erase does
 * the same until power_on(). A test can therefore sweep the cut across
 * every single write of a mutation and assert, at each of them, that the
 * heap comes back as either entirely the old thing or entirely the new
 * one.
 *
 * WHAT IT MODELS OF THE BENCH. reflash() is the avrdude -D convention:
 * an image is written over the pages it occupies and the rest of the
 * part is left alone - the mechanism the whole survival story rests on.
 * chip_erase() is the other half of the truth: everything goes.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <array>
#include <span>

#include "util/nv_heap.hpp"

namespace brio {

/**
 * A simulated flash part. Every member is static: like the silicon it
 * stands for, there is exactly one of it, and the geometry is in the
 * type so two geometries are two types.
 *
 *   Erase      bytes in an erase unit
 *   Cell       bytes in a program unit
 *   Size       bytes in the whole part
 *   ZoneCount  usable bands the media reports (their bounds are set at
 *              run time, as they are on a real target)
 */
template <uint32_t Erase = 512, uint32_t Cell = 2, uint32_t Size = 128u * 1024u,
          uint8_t ZoneCount = 2>
struct SimFlash {
    static_assert(Size % Erase == 0);
    static_assert(Erase % Cell == 0);
    static_assert(ZoneCount >= 1);

    // ---- the FlashMedia surface --------------------------------------------

    static constexpr uint32_t erase_size = Erase;
    static constexpr uint32_t write_cell = Cell;
    static constexpr uint32_t flash_end = Size;
    static constexpr uint8_t zone_count = ZoneCount;

    static constexpr uint32_t page_count = Size / Erase;
    static constexpr uint32_t cell_count = Size / Cell;
    static constexpr uint32_t unlimited = 0xFFFFFFFFu;

    static std::array<FlashZone, ZoneCount> zones() { return zone_table; }

    static void read(uint32_t addr, std::span<uint8_t> dst) {
        ++reads;
        for (uint32_t i = 0; i < dst.size(); ++i) {
            dst[i] = (addr + i) < Size ? bytes[addr + i] : 0xFFu;
        }
    }

    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        ++program_calls;
        if (addr % Cell != 0 || src.size() % Cell != 0 ||
            addr + src.size() > Size) {
            ++misaligned;
            return false;
        }
        if (!powered) {
            return false;
        }
        for (uint32_t off = 0; off < src.size(); off += Cell) {
            if (budget == 0) {
                powered = false;    // the lights go out mid-write
                return false;
            }
            if (budget != unlimited) {
                --budget;
            }
            const uint32_t at = addr + off;
            if (!fresh[at / Cell]) {
                ++double_programs;
            }
            fresh[at / Cell] = false;
            for (uint32_t i = 0; i < Cell; ++i) {
                bytes[at + i] &= src[off + i];   // programming only clears bits
            }
            ++cells_programmed;
        }
        return true;
    }

    static bool erase(uint32_t addr) {
        ++erase_calls;
        if (addr % Erase != 0 || addr + Erase > Size) {
            ++misaligned;
            return false;
        }
        if (!powered) {
            return false;
        }
        wipe(addr);
        ++erase_count[addr / Erase];
        return true;
    }

    static uint32_t build_id() { return id; }

    // ---- the bench controls -------------------------------------------------

    /// A virgin part with fresh counters, no zones and the power on.
    static void reset() {
        chip_erase();
        for (uint32_t i = 0; i < page_count; ++i) {
            erase_count[i] = 0;
        }
        for (uint8_t i = 0; i < ZoneCount; ++i) {
            zone_table[i] = FlashZone{0, 0};
        }
        double_programs = 0;
        cells_programmed = 0;
        program_calls = 0;
        erase_calls = 0;
        misaligned = 0;
        reads = 0;
        budget = unlimited;
        powered = true;
        id = 0;
    }

    /// Everything goes - the UPDI chip erase, or avrdude without -D.
    static void chip_erase() {
        for (uint32_t a = 0; a < Size; a += Erase) {
            wipe(a);
        }
    }

    /// The avrdude -D convention: the image is written over the pages it
    /// occupies (which are erased first, and pay for it), and every other
    /// page of the part is left exactly as it was. This is the mechanism
    /// a heap outside the image survives on.
    static void reflash(uint32_t addr, std::span<const uint8_t> image) {
        const uint32_t end = addr + image.size();
        for (uint32_t a = addr / Erase * Erase; a < end; a += Erase) {
            wipe(a);
            ++erase_count[a / Erase];
        }
        for (uint32_t i = 0; i < image.size(); ++i) {
            bytes[addr + i] = image[i];
            fresh[(addr + i) / Cell] = false;
        }
    }

    /// Let n more program units land, then lose the power.
    static void cut_after(uint32_t n) {
        budget = n;
        powered = true;
    }
    /// Back from the dead, with the flash exactly as the cut left it.
    static void power_on() {
        powered = true;
        budget = unlimited;
    }

    static void set_zone(uint8_t i, uint32_t ceiling, uint32_t floor) {
        zone_table[i] = FlashZone{ceiling, floor};
    }
    static void set_build_id(uint32_t v) { id = v; }

    static uint32_t erases_of_page(uint32_t page) { return erase_count[page]; }
    static uint32_t total_erases() {
        uint32_t n = 0;
        for (uint32_t i = 0; i < page_count; ++i) {
            n += erase_count[i];
        }
        return n;
    }

    // ---- the state itself ----------------------------------------------------

    static inline std::array<uint8_t, Size> bytes{};
    /// Has this program unit been left alone since its last erase? The
    /// ECC discipline, tracked rather than enforced.
    static inline std::array<bool, cell_count> fresh{};
    static inline std::array<uint32_t, page_count> erase_count{};
    static inline std::array<FlashZone, ZoneCount> zone_table{};

    static inline uint32_t double_programs = 0;  ///< the ECC violations
    static inline uint32_t cells_programmed = 0;
    static inline uint32_t program_calls = 0;
    static inline uint32_t erase_calls = 0;
    static inline uint32_t misaligned = 0;       ///< refused for alignment/bounds
    static inline uint32_t reads = 0;
    static inline uint32_t budget = unlimited;
    static inline bool powered = true;
    static inline uint32_t id = 0;

private:
    static void wipe(uint32_t addr) {
        for (uint32_t i = 0; i < Erase; ++i) {
            bytes[addr + i] = 0xFFu;
        }
        for (uint32_t i = 0; i < Erase / Cell; ++i) {
            fresh[addr / Cell + i] = true;
        }
    }
};

} // namespace brio
