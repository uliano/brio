/*
 * nvm.hpp
 *
 * The flash program/erase engine of the CH32V00x (RM ch. 18): the
 * resource the storage media in nvm_flash.hpp are built on, and the
 * one place in the stratum that writes FLASH_CTLR.
 *
 * ONE PROGRAMMING GRANULARITY, THE PAGE. This family programs its main
 * memory by FAST PAGE PROGRAMMING only: 256 bytes go into an internal
 * buffer as 64 words, each word followed by a BUFLOAD, and one STRT
 * programs the page (18.4.5). There is no half-word PG mode - the
 * CTLR has no such bit - so the smallest thing this engine can write
 * is a whole page, and the FlashMedia above it says `write_cell =
 * 256`. Erasing has three grains: the fast page of 256 bytes (FTER),
 * the standard sector of 1 KB (PER), the whole array (MER); the media
 * erase in pages, so erase_size and write_cell coincide here, which is
 * unusual and stated where it matters.
 *
 * TWO LOCKS, ONE WINDOW. The FPEC lock (KEYR, LOCK) guards CTLR and
 * the standard operations; the fast lock (MODEKEYR, FLOCK) guards the
 * fast ones on top, and each needs its own key pair in order and
 * consecutively - a wrong sequence locks the block until the next
 * system reset (18.4.2, 18.4.4). unlock() opens both and lock()
 * closes both; the media keep the window to the one operation.
 *
 * HSI MUST BE ON while programming or erasing (18.2.2's note); every
 * clock this stratum offers keeps it on (clock.hpp: the PLL's source,
 * or the root itself).
 *
 * THE ADDRESS THE ENGINE TAKES is the 0x0800 0000 alias of the array,
 * the one the chapter's own example writes through (18.4.5 step 7);
 * the image runs from the array at address 0 and the media below
 * number their bytes from 0 too, so `alias_of()` is the one
 * translation, applied here and nowhere else.
 *
 * WHAT IS NOT HERE: the option bytes (OBKEYR/OBER/OBWRE, 18.5) and the
 * 2 KB write-protection units (WPR) as verbs - both arrive with the
 * `brio fuses` work for this target; the 32 KB block erase (BER32),
 * which has no user; the BOOT area.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <span>

#include "ch32v00x/device.hpp"

namespace brio {

struct Flash {
    Flash() = delete;

    static constexpr uint32_t page_size = 256;      ///< the fast program AND erase unit
    static constexpr uint32_t sector_size = 1024;   ///< the standard erase unit
    static constexpr uint32_t alias_base = 0x08000000UL;
    static constexpr uint32_t array_bytes = 62u * 1024u;   ///< the CH32V006K8's

    /// How long an operation may take before the engine gives up, in
    /// polls: a page erase is a few milliseconds, and a stuck BSY is a
    /// fact the caller must see rather than a hang.
    static constexpr uint32_t timeout_polls = 2'000'000UL;

    /// What a verb returns when the CALLER broke its contract (a
    /// misaligned address, a span that is not a page): a bit no STATR
    /// error uses, so a refusal is never mistaken for a silicon error.
    static constexpr uint32_t refused = 1UL << 31;

    static constexpr uint32_t alias_of(uint32_t addr) { return alias_base + addr; }

    static bool locked() { return (flash_ctl()->CTLR & flash_lock) != 0u; }
    static bool fast_locked() { return (flash_ctl()->CTLR & flash_flock) != 0u; }
    static bool busy() { return (flash_ctl()->STATR & flash_bsy) != 0u; }

    /**
     * Open both locks. False when either stays shut - which, after a
     * wrong sequence, is until the next reset (18.4.2), so a false here
     * is not something to retry.
     */
    static bool unlock() {
        if (locked()) {
            flash_ctl()->KEYR = flash_key1;
            flash_ctl()->KEYR = flash_key2;
        }
        if (fast_locked()) {
            flash_ctl()->MODEKEYR = flash_key1;
            flash_ctl()->MODEKEYR = flash_key2;
        }
        return !locked() && !fast_locked();
    }

    /// Close both locks: writing 1 to each is what sets them (RW1).
    static void lock() {
        flash_ctl()->CTLR = flash_ctl()->CTLR | flash_flock | flash_lock;
    }

    /// Read: the array is memory-mapped, at 0 and at the alias.
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        memcpy(dst.data(), reinterpret_cast<const void*>(alias_of(addr)), dst.size());
    }

    /**
     * Erase one 256-byte page at `addr` (page-aligned), the fast way
     * (18.4.6). Returns the error bits of STATR (WRPRTERR) or zero; the
     * timeout counts as an error too. Both locks must be open.
     */
    static uint32_t erase_page(uint32_t addr) {
        if ((addr % page_size) != 0u) {
            return refused;
        }
        if (!wait_idle()) {
            return flash_bsy;
        }
        clear_flags();
        flash_ctl()->CTLR = flash_ctl()->CTLR | flash_fter;
        flash_ctl()->ADDR = alias_of(addr);
        flash_ctl()->CTLR = flash_ctl()->CTLR | flash_strt;
        const uint32_t err = finish();
        flash_ctl()->CTLR = flash_ctl()->CTLR & ~flash_fter;
        return err;
    }

    /// Erase one 1 KB sector at `addr` (sector-aligned), the standard
    /// way (18.4.3). Same contract as erase_page().
    static uint32_t erase_sector(uint32_t addr) {
        if ((addr % sector_size) != 0u) {
            return refused;
        }
        if (!wait_idle()) {
            return flash_bsy;
        }
        clear_flags();
        flash_ctl()->CTLR = flash_ctl()->CTLR | flash_per;
        flash_ctl()->ADDR = alias_of(addr);
        flash_ctl()->CTLR = flash_ctl()->CTLR | flash_strt;
        const uint32_t err = finish();
        flash_ctl()->CTLR = flash_ctl()->CTLR & ~flash_per;
        return err;
    }

    /**
     * Program one whole page at `addr` (page-aligned) from `src`, which
     * must be exactly page_size bytes (18.4.5): reset the buffer, load
     * 64 words through it, then start. Same contract as erase_page().
     * The words are written through the alias, as the chapter's own
     * example does.
     */
    static uint32_t program_page(uint32_t addr, std::span<const uint8_t> src) {
        if (src.size() != page_size || (addr % page_size) != 0u) {
            return refused;
        }
        if (!wait_idle()) {
            return flash_bsy;
        }
        clear_flags();
        flash_ctl()->CTLR = flash_ctl()->CTLR | flash_ftpg;
        flash_ctl()->CTLR = flash_ctl()->CTLR | flash_bufrst;
        if (!wait_idle()) {
            flash_ctl()->CTLR = flash_ctl()->CTLR & ~flash_ftpg;
            return flash_bsy;
        }
        clear_flags();
        volatile uint32_t* cell = reinterpret_cast<volatile uint32_t*>(alias_of(addr));
        for (uint32_t i = 0; i < page_size / 4u; ++i) {
            uint32_t word;
            memcpy(&word, src.data() + i * 4u, 4u);
            cell[i] = word;
            flash_ctl()->CTLR = flash_ctl()->CTLR | flash_bufload;
            if (!wait_idle()) {
                flash_ctl()->CTLR = flash_ctl()->CTLR & ~flash_ftpg;
                return flash_bsy;
            }
        }
        flash_ctl()->ADDR = alias_of(addr);
        flash_ctl()->CTLR = flash_ctl()->CTLR | flash_strt;
        const uint32_t err = finish();
        flash_ctl()->CTLR = flash_ctl()->CTLR & ~flash_ftpg;
        return err;
    }

    /// FLASH_WPR as loaded from the option bytes: one bit per 2 KB unit,
    /// a clear bit meaning PROTECTED.
    static uint32_t write_protection() { return flash_ctl()->WPR; }

    static uint32_t status() { return flash_ctl()->STATR; }

private:
    static bool wait_idle() {
        for (uint32_t i = 0; i < timeout_polls; ++i) {
            if (!busy()) {
                return true;
            }
        }
        return false;
    }

    /// EOP and WRPRTERR are write-1-clear.
    static void clear_flags() {
        flash_ctl()->STATR = flash_eop | flash_wrprterr;
    }

    /// Wait for the operation, then read what it left: the error bit,
    /// or zero. EOP is cleared on the way out.
    static uint32_t finish() {
        if (!wait_idle()) {
            return flash_bsy;
        }
        const uint32_t err = flash_ctl()->STATR & flash_wrprterr;
        clear_flags();
        return err;
    }
};

} // namespace brio
