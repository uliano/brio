/*
 * nvm_flash.hpp
 *
 * The SAM C21 flash as a FlashMedia (util/nv_heap.hpp): the backend that
 * lets the target-independent block allocator run on this silicon, and
 * the second implementation of that contract - the first one written
 * against an architecture the concept was not designed on.
 *
 * IT LIVES IN THE RWWEE ARRAY, and that choice is the whole design.
 *
 * This family has two programmable arrays: the 256 KB main array the CPU
 * executes from, and a dedicated 8 KB Read-While-Write EEPROM array at
 * 0x00400000. Writing or erasing the MAIN array stalls the AHB - every
 * instruction fetch with it - until the operation ends, exactly the way
 * an AVR page erase halts that CPU for its full 10 ms. Writing the RWWEE
 * array stalls NOTHING: the main array can be read throughout (27.6.4.1),
 * so a program that stores a record keeps running while it is stored.
 *
 * That difference removes the AVR backend's hardest problem as a
 * side effect. There, the free flash of a linked image is two bands whose
 * inner edges are linker symbols moving with every build, and an image
 * that grows silently lands on stored blocks (which is why tools/bench.py
 * grew a preflight). The RWWEE array is not part of the link at all:
 * nothing the compiler emits can ever land in it, so the zone is a
 * CONSTANT - the whole array - and no linker symbol is read here.
 *
 * The price is size and speed: 8 KB rather than the tens of kilobytes a
 * main-array heap could reach, and reads that miss the cache because the
 * RWWEE array is not cached (27.6.4.2). For a heap holding calibration,
 * counters and panic records, that is the right trade; a main-array
 * backend is a separate type when something needs the room (see
 * docs/samc/nvm.md, "Not covered yet").
 *
 * THE GRANULARITIES ARE NOT THE SAME NUMBER, which is what the concept's
 * erase_size / write_cell split is for: here an erase takes down a ROW of
 * 256 bytes and a program writes a PAGE of 64. On the AVR they are 512
 * and 2. Code that says "page" for both is code that is wrong on one of
 * the two targets - and note that on THIS target "page" is the small one,
 * the opposite of the AVR's usage, which is the trap the split exists to
 * make harmless.
 *
 * ADDRESSES ARE ABSOLUTE. NvHeap works in flat byte addresses and derives
 * an erase-unit index from them, so the heap's `first_page` numbers here
 * start at 0x00400000 / 256 = 16384 rather than at 0 - which is exactly
 * why the index is a page number and not an offset.
 *
 * THE BUILD ID is a link-time constant, as on the AVR:
 * samc/CMakeLists.txt passes -Wl,--defsym,__nvheap_build_id=<newest
 * source mtime> to every image. A wall-clock id would make every relink a
 * different image; the newest source timestamp keeps an unchanged tree
 * relinking to the same bytes. Unlike the AVR, reading it needs no
 * assembly: a pointer is 32 bits here, so the symbol's address IS the
 * value. It is recorded in every map version and NOTHING decides on it -
 * a block's validity is its checksum's business.
 */

#pragma once

#include <stdint.h>

#include <array>
#include <span>

#include "samc/nvm.hpp"
#include "util/nv_heap.hpp"

extern "C" {
/// The epoch of the link, injected by samc/CMakeLists.txt. Declared as an
/// array so that no code can be tempted to dereference it: it is a VALUE,
/// and only its address bits are ever read.
extern const char __nvheap_build_id[];
}

namespace brio {

/// The RWWEE array behind the FlashMedia contract. All static: there is
/// one flash, and it is a piece of hardware.
struct RwweeFlash {
    RwweeFlash() = delete;

    static constexpr uint32_t erase_size = Nvm::row_size;    // 256
    static constexpr uint32_t write_cell = Nvm::page_size;   // 64
    static constexpr uint32_t flash_end = Nvm::rwwee_end;
    static constexpr uint8_t zone_count = 1;

    /// The one zone: the whole array. Constant, unlike every zone on the
    /// AVR side - nothing the linker places can reach in here, so there
    /// is no image edge to round and no build that can move it. The heap
    /// carves its own map home out of the top of this by itself.
    static std::array<FlashZone, zone_count> zones() {
        return std::array<FlashZone, zone_count>{
            FlashZone{flash_end, Nvm::rwwee_base}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) {
        Nvm::read(addr, dst);
    }

    /**
     * Program whole pages. NvHeap calls this in write_cell units, so the
     * loop below normally runs once; it is a loop because the contract
     * allows a multiple.
     *
     * Every page goes through Nvm::program_page, which owns the two page
     * -buffer rules (32-bit stores, strictly ascending) and the command
     * discipline. This does NOT stall the CPU: that is the point of
     * living in the RWWEE array.
     */
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        if ((addr % write_cell) != 0u || (src.size() % write_cell) != 0u) {
            return false;
        }
        for (uint32_t off = 0; off < src.size(); off += write_cell) {
            if (Nvm::program_page(NvmArray::rwwee, addr + off,
                                  src.subspan(off, write_cell)) !=
                NvmError::none) {
                return false;
            }
        }
        return true;
    }

    /// One row. The RWWEE array has no lock regions - its rows are
    /// writable regardless of the main array's region locks - so the only
    /// failures here are an address outside the array and a controller
    /// error.
    static bool erase(uint32_t addr) {
        return Nvm::erase_row(NvmArray::rwwee, addr) == NvmError::none;
    }

    /// The epoch of the link. Diagnostic only.
    static uint32_t build_id() {
        return static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(&__nvheap_build_id[0]));
    }
};

static_assert(FlashMedia<RwweeFlash>);

// The heap's own static_asserts check most of this, but these two are
// this target's specifically and are worth failing on by name.
static_assert(RwweeFlash::flash_end % RwweeFlash::erase_size == 0u,
              "the map home is anchored to flash_end");
static_assert(RwweeFlash::flash_end / RwweeFlash::erase_size <= 0xFFFFu,
              "NvHeap numbers erase units in a uint16_t, and RWWEE addresses "
              "are absolute - the array must not sit past page 65535");

} // namespace brio
