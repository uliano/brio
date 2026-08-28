/*
 * nvm_flash.hpp
 *
 * The AVR DA/DB flash as a FlashMedia (util/nv_heap.hpp): the backend
 * that lets the target-independent block allocator run on this silicon.
 *
 * THE GRANULARITIES ARE NOT THE SAME NUMBER. An erase takes down a
 * 512-byte page; a program writes ONE 16-BIT WORD, and a word may only
 * be written once between erases. That is why the concept keeps the two
 * apart - on this part they are 512 and 2, on an STM32G0 they are 2048
 * and 8, and code that says "page" for both is code that will be wrong
 * on the second target.
 *
 * ADDRESSES ARE 24-BIT AND GO THROUGH ELPM/SPM, never through the
 * data-space FLMAP window - the discipline avrdx/nvm.hpp is built on and
 * the reason the DA's FLMAP-vs-protection erratum is inapplicable here
 * by construction. Every verb below is one call into that driver, which
 * keeps its own validation (alignment, device bounds, the section
 * geometry from the fuses, the protection bits, the whole-range check
 * the multi-page-erase erratum makes necessary).
 *
 * THE ZONES, AND WHERE THEIR BOUNDS COME FROM. gcc puts the code and the
 * .data initializers low and .rodata in a 32 KB flash section reached
 * through the window (96 KB in on a 128 KB part), so the free flash of a
 * linked image is not one remainder at the top but TWO bands, and both
 * of their inner edges are linker symbols that move with every build:
 *
 *   MIDDLE  ceiling  __rodata_load_start, rounded down to a page
 *           floor    __data_load_end rounded up to a page, RAISED TO THE
 *                    BOOT BOUNDARY - SPM cannot write inside BOOT
 *                    whatever the fuses say, and with the bench geometry
 *                    (BOOTSIZE = 128) all the code is in BOOT anyway
 *   TAIL    ceiling  the end of the flash (the heap carves its map home
 *                    out of the top of this zone by itself)
 *           floor    __rodata_load_end, rounded up to a page
 *
 * The symbols are the LOAD addresses, which on this toolchain are real
 * flash addresses; the RUN-time twins of the same data (__rodata_start
 * and friends) are data-space aliases based at 0x800000 and would be
 * nonsense here. Only the *_load_* ones are read, and always through
 * pgm_get_far_address, because a plain &symbol is 16 bits wide on this
 * part and would silently lose the top of the memory.
 *
 * THE BUILD ID is a link-time constant, not a compile-time one:
 * CMakeLists.txt's avr_add_app() passes -Wl,--defsym,__nvheap_build_id=<newest
 * source mtime> to every image, the same way it locks FLMAP (a
 * wall-clock id would make every relink a different image and defeat
 * reflashing an unchanged one). It is read as a 32-bit
 * IMMEDIATE (the four relocation bytes of the symbol's value), because
 * the value is a number and not an address - taking &symbol would
 * truncate it to the 16 bits a pointer has here. It is recorded in every
 * map version and NOTHING decides on it: a block's validity is its
 * checksum's business.
 */

#pragma once

#include <stdint.h>

#include <array>
#include <span>

#include "avrdx/nvm.hpp"
#include "util/nv_heap.hpp"

extern "C" {
/// The epoch of the link, injected by CMakeLists.txt's avr_add_app(). Declared as an
/// array so that no code can be tempted to dereference it: it is a
/// VALUE, and only its relocation bytes are ever read.
extern const char __nvheap_build_id[];
}

namespace brio {

/// The flash of this family behind the FlashMedia contract. All static:
/// there is one flash, and it is a piece of hardware.
struct NvmFlash {
    NvmFlash() = delete;

    static constexpr uint32_t erase_size = flash_page_size;
    static constexpr uint32_t write_cell = 2;
    static constexpr uint32_t flash_end = flash_size;
    static constexpr uint8_t zone_count = 2;

    /// The two bands of free flash, as the image linked today leaves
    /// them. A runtime call on purpose: the bounds are the linker's, and
    /// they move with every build.
    static std::array<FlashZone, zone_count> zones() {
        const uint32_t image = page_up(Nvm::image_low_end());
        const uint32_t boot = Nvm::boot_end();
        const uint32_t middle_floor = image > boot ? image : boot;
        const uint32_t middle_ceiling = page_down(Nvm::rodata_load_start());
        return std::array<FlashZone, zone_count>{
            FlashZone{middle_ceiling < middle_floor ? middle_floor
                                                    : middle_ceiling,
                      middle_floor},
            FlashZone{flash_end, page_up(Nvm::rodata_load_end())}};
    }

    static void read(uint32_t addr, std::span<uint8_t> dst) {
        for (uint32_t i = 0; i < dst.size(); ++i) {
            dst[i] = Nvm::flash_read(addr + i);
        }
    }

    /// One run of words with FLWR selected once (11.3.2.3.1). The CPU is
    /// halted for each word - about 83 us measured, see nvm.md - so a
    /// program of a whole page is a decision about the system's latency
    /// and not a library call.
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        if ((addr & 1u) != 0 || (src.size() & 1u) != 0 ||
            src.size() > 0xFFFEu) {
            return false;
        }
        return Nvm::write_block(addr, src.data(),
                                static_cast<uint16_t>(src.size()));
    }

    /// One page, with the driver's own whole-range protection check. The
    /// CPU is halted for the full ~10 ms.
    static bool erase(uint32_t addr) { return Nvm::erase(addr, FlashErase::page); }

    /// The epoch of the link, as four relocation bytes. Diagnostic only.
    static uint32_t build_id() {
        uint32_t v;
        __asm__("ldi %A0, lo8(__nvheap_build_id)\n\t"
                "ldi %B0, hi8(__nvheap_build_id)\n\t"
                "ldi %C0, hlo8(__nvheap_build_id)\n\t"
                "ldi %D0, hhi8(__nvheap_build_id)"
                : "=d"(v));
        return v;
    }

private:
    static constexpr uint32_t page_up(uint32_t v) {
        return (v + erase_size - 1u) / erase_size * erase_size;
    }
    static constexpr uint32_t page_down(uint32_t v) {
        return v / erase_size * erase_size;
    }
};

static_assert(FlashMedia<NvmFlash>);

} // namespace brio
