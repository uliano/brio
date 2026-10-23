/*
 * nvm.hpp
 *
 * The flash program/erase engine of the CH32V00x (RM ch. 18): the
 * resource the storage media in nvm_flash.hpp are built on, and the
 * one place in the stratum that writes FLASH_CTLR. A program rarely
 * calls it: it keeps its data through util/nv_heap.hpp's heap or
 * util/nv_journal.hpp's journal over those media
 * (docs/ch32v00x/nvm.md, "How to use it"). A caller of the engine
 * itself opens the locks, runs one operation and shuts them:
 *
 *     if (Flash::unlock()) {
 *         const uint32_t err = Flash::program_page(addr, page);   // 0 = done
 *         Flash::lock();
 *     }
 *
 * Who calls what, and what waits. `Flash` is a monostate (a type whose
 * functions are all static: one engine, never an object), called from
 * main or one active object, never from an interrupt handler, with no
 * guard of its own. Every operation waits: the core executes from this
 * array and there is no read-while-write, so a page erase or a program
 * stalls the core for its duration - 2.9 ms and 1.9 ms measured on the
 * CH32V006K8U6 (docs/ch32v00x/nvm.md) - and every interrupt waits with it; the polls
 * for BSY around the stall are bounded (timeout_polls), and a BSY that
 * never falls is reported, not waited out. Nothing masks interrupts.
 * The caller's buffer is read during program_page() and free when it
 * returns. Every address the engine takes counts from 0, the array's
 * own; `alias_of()` adds the 0x0800 0000 alias inside each verb.
 *
 * One programming granularity, the page. This family programs its main
 * memory by fast page programming: the page's words go into an internal
 * buffer, each word followed by a BUFLOAD, and one STRT programs the page
 * (18.4.5) - 256 bytes as 64 words on the CH32V006, 64 bytes as 16 words
 * on the CH32V003 (page_size, device::flash_page_bytes). The CH32V006's
 * CTLR has no half-word PG mode; the CH32V003's has the standard one, which
 * this engine does not use - so the smallest thing this engine writes is a
 * whole page on both parts, and the FlashMedia above it says `write_cell =`
 * the page. Erasing has three grains: the fast page (FTER), the standard
 * sector of 1 KB (PER), the whole array (MER); the media erase in pages,
 * so erase_size and write_cell coincide here, which is unusual and stated
 * where it matters.
 *
 * Two locks, one window. The FPEC lock (KEYR, LOCK) guards CTLR and
 * the standard operations; the fast lock (MODEKEYR, FLOCK) guards the
 * fast ones on top, and each needs its own key pair in order and
 * consecutively - a wrong sequence locks the block until the next
 * system reset (18.4.2, 18.4.4). unlock() opens both and lock()
 * closes both; the media keep the window to the one operation.
 *
 * The HSI must be on while programming or erasing (18.2.2's note): no
 * clock type of clock.hpp turns it off (it is the PLL's source, or the
 * root itself, or left running beside the crystal), while Rcc's HSI verb
 * can, and nothing here checks it.
 *
 * The address written into FLASH_ADDR, and the cells the words are
 * stored through, are the 0x0800 0000 alias of the array, the one the
 * chapter's own example writes through (18.4.5 step 7); the image runs
 * from the array at address 0 and the media below number their bytes
 * from 0 too, so `alias_of()` is the one translation, applied here and
 * nowhere else.
 *
 * What is not here: the option bytes (OBKEYR/OBER/OBWRE, 18.5) and the
 * 2 KB write-protection units (WPR) as verbs - a wrong write costs a
 * chip erase, and they are provisioning, which on the AVR is the bench
 * verb `brio fuses`, a verb with no CH32V00x side (docs/ch32v00x/nvm.md's
 * gap list; WPR is read by write_protection()); the 32 KB block erase
 * (BER32), which has no user; the BOOT area.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <span>

#include "ch32v00x/device.hpp"

namespace brio {

struct Flash {
    Flash() = delete;

    static constexpr uint32_t page_size = device::flash_page_bytes;   ///< the fast program AND erase unit: 256 on the CH32V006, 64 on the CH32V003
    static constexpr uint32_t sector_size = 1024;   ///< the standard erase unit
    static constexpr uint32_t alias_base = 0x08000000UL;
    static constexpr uint32_t array_bytes = device::flash_bytes;   ///< 62 KB on the CH32V006 (the boot area apart), 16 on the CH32V003

    /// How long an operation may take before the engine gives up, in
    /// polls: a page erase is a few milliseconds, and a stuck BSY is a
    /// fact the caller must see rather than a hang.
    static constexpr uint32_t timeout_polls = 2'000'000UL;

    /// What a verb returns when the caller broke its contract (a
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
     * Erase one page at `addr` (page-aligned: 256 bytes on the
     * CH32V006, 64 on the CH32V003), the fast way (18.4.6). Returns the
     * error bits of STATR (WRPRTERR) or zero; the timeout counts as an
     * error too (BSY's bit), and a misaligned address is `refused`.
     * Both locks must be open. The core stalls for the erase.
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
     * the page's words through it (64 on the CH32V006, 16 on the
     * CH32V003), then start. Same contract as erase_page(); `src` is
     * read here and free when the call returns.
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
    /// a clear bit meaning protected.
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
