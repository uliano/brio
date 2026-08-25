/*
 * nvm.hpp
 *
 * NVMCTRL (chapter 11): the one peripheral that changes the program
 * itself. Flash, EEPROM, the User Row, the Signature Row and the fuses
 * all sit behind it, and every one of them has a different granularity,
 * a different command and a different way of going wrong.
 *
 * THREE MEMORIES, THREE JOBS (the division this driver is built around):
 *
 *  - FLASH is big and cheap to read and expensive to keep: 512-byte
 *    pages, 1000 erase/write cycles per cell, and a chip erase - every
 *    reflash - takes it all away. It is where a re-provisionable
 *    payload lives: a font, a table, a calibration set that the
 *    programmer can put back.
 *  - EEPROM is small, byte-granular and 100 times more durable
 *    (512 bytes, 100k cycles), and with the EESAVE fuse set it survives
 *    a chip erase. It is where the settings live.
 *  - USERROW is 32 bytes that survive a chip erase ALWAYS, fuse or no
 *    fuse, and can be written over UPDI even on a locked device. It is
 *    identity and provisioning, not runtime state (avrdx/userrow.hpp
 *    reads the board label out of it).
 *
 * HOW THE FLASH IS REACHED, AND WHY. The Flash is visible twice: as
 * code space through LPM/SPM with RAMPZ:Z, and as a 32 KB WINDOW in
 * data space whose position is chosen by NVMCTRL.CTRLB.FLMAP. This
 * driver uses the CODE SPACE ONLY - ELPM to read, SPM to write, never
 * an LD or ST through the window and never a write to FLMAP. Two
 * reasons, and both are load-bearing:
 *
 *   1. DS80000882C 2.7.1 (AVR DA): the inter-section write protection
 *      does not take FLMAP into account, so with FLMAP != 0 the BOOT
 *      section is MIRRORED into every mapped section and a write that
 *      the hardware thinks is legal lands somewhere else. The errata's
 *      own work-around is "use only SPM to write and LPM to read".
 *      Following it by construction makes the whole item inapplicable
 *      here rather than something to remember.
 *   2. A window is a mode. A 24-bit address that is always an address
 *      needs no mode, and `&symbol` truncated to 16 bits - the classic
 *      way to lose the top of a 128 KB part - cannot happen when every
 *      verb takes a uint32_t.
 *
 * FLMAP itself is not hidden: flmap()/flmap_locked() report it and
 * set_flmap()/lock_flmap() drive it, because an app may legitimately
 * want the window for its own READS (that half of the DA erratum is
 * explicitly fine) and because the C runtime already uses it - gcc
 * places .rodata in a flash section and emits a FLMAP write in .init.
 * No verb in this file consults it.
 *
 * THE ERRATA THAT ARE CODE HERE. DS80000915F 2.7.1 / DS80000882C 2.7.2,
 * both families, all revisions on this desk: a MULTI-PAGE erase checks
 * only the FIRST page of the range against write protection and erases
 * the rest regardless. There is no work-around in silicon, so the
 * driver validates the WHOLE range - every page of it - against the
 * section geometry and the protection bits before issuing any erase.
 * What the hardware would wrongly allow, erase() refuses.
 *
 * DS80000915F 2.7.2 says the EEPROM erase command ignores an "EEWP" bit
 * in CTRLB. No such bit exists on AVR DA or DB: CTRLB is FLMAPLOCK,
 * FLMAP, APPDATAWP, BOOTRP, APPCODEWP and nothing else (11.5.2, and the
 * device headers of all eight packages agree). The practical statement
 * is the one worth remembering: the EEPROM on this family has NO write
 * protection at all - the section protections cover Flash only.
 *
 * WHAT IS DELIBERATELY NOT HERE. CHER (chip erase) and EECHER (EEPROM
 * erase) are not exposed. CHER is UPDI-only in the silicon (11.3.2.3.8)
 * and EECHER would let one line of application code destroy every
 * setting the product has; erasing a whole memory is a provisioning
 * act, and provisioning happens over the programmer. The fuses are not
 * writable at all from software (11.3.1.5): the CPU reads them, only
 * UPDI programs them.
 *
 * COMMAND DISCIPLINE (11.3.2.3). CTRLA is CCP-protected with the SPM
 * key, and a command must go through NOCMD/NOOP before another one is
 * selected - selecting a new command over a live one raises an ERROR
 * instead of doing anything. Every verb here follows the chapter's
 * sequence: wait for the busy flags, clear the command, select the new
 * one, do the store, clear the command again. CTRLB's other half is
 * CCP-protected too, with the IOREG key, EXCEPT FLMAP[1:0] which the
 * chapter says explicitly is not (11.5.2).
 *
 * WHO STALLS WHOM (table 11-1). A Flash erase or write HALTS THE CPU
 * for the whole operation: ten milliseconds for a page erase, seventy
 * microseconds for a word. Nothing runs, interrupts included - a flash
 * write is a decision about the whole system's latency, not a library
 * call. An EEPROM write does NOT halt the CPU; the CPU is halted only
 * if it starts a second load/store into NVM while the first is still
 * running, which is what makes an interrupt-paced writer possible at
 * all (util/nv_writer.hpp).
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "util/nv_record.hpp"

extern "C" {
/// LMA just past .text and the .data initializers, and the LMAs of the
/// image's read-only data. All three are defined by the AVR linker
/// script and all three sit above 64 KB on a part this size, so only
/// their link-time ADDRESSES are ever taken, never their contents, and
/// always through pgm_get_far_address: a plain `&symbol` is 16 bits
/// wide here and would silently truncate.
extern const char __data_load_end[];
extern const char __rodata_load_start[];
extern const char __rodata_load_end[];
}

namespace brio {

// =============================================================================
// Geometry, from the device header
// =============================================================================

/// The whole Flash, in bytes (128 KB on every AVR128DA/DB package).
inline constexpr uint32_t flash_size = PROGMEM_SIZE;
/// The erase granularity of the Flash: one page, 512 bytes (11.3.1.1).
inline constexpr uint16_t flash_page_size = PROGMEM_PAGE_SIZE;
/// The block the BOOTSIZE/CODESIZE fuses count in (11.3.1.1).
inline constexpr uint16_t flash_section_block = 512;

inline constexpr uint16_t eeprom_size = EEPROM_SIZE;
inline constexpr uint16_t eeprom_base = EEPROM_START;

inline constexpr uint8_t userrow_size = USER_SIGNATURES_SIZE;
inline constexpr uint16_t userrow_base = USER_SIGNATURES_START;

/// The three Flash sections (11.3.1.1). Which addresses are in which is
/// a FUSE fact, so it is answered at run time by Nvm::section_of() and
/// at compile time by FlashLayout.
enum class FlashSection : uint8_t {
    boot,     ///< full write access from a programmer, none from the CPU
    appcode,  ///< writable from code executing in BOOT
    appdata,  ///< writable from code executing in BOOT or APPCODE
};

/// NVMCTRL.STATUS.ERROR (11.5.3). The data sheet lists three codes; the
/// device headers of every package list FIVE, splitting the data
/// sheet's single CMDCOLLISION into "a command was selected while one
/// was already selected" (0x3) and "a programming operation was started
/// before the previous one finished" (0x4). The names below are the
/// data sheet's where the two agree and the header's where they do not.
enum class NvmError : uint8_t {
    none = 0,
    invalid_command = 1,    ///< INVALIDCMD / ILLEGALCMD
    write_protect = 2,      ///< WRITEPROTECT / ILLEGALSADDR
    command_collision = 3,  ///< CMDCOLLISION / DOUBLESELECT
    ongoing_program = 4,    ///< ONGOINGPROG (header only)
};

/**
 * NVMCTRL.CTRLA.CMD (11.5.1), the chapter's own two-step model: a
 * command is SELECTED here, and then carried out by an ordinary store
 * into the memory array. Every verb in this driver does both steps for
 * you; the field is exposed because a caller doing a long run of writes
 * may legitimately want to select once and store many times, which is
 * what 11.3.2.3 step 5 says to do.
 *
 * TWO COMMANDS ARE MISSING ON PURPOSE. CHER erases the whole Flash AND
 * the EEPROM and is UPDI-only in the silicon anyway (11.3.2.3.8);
 * EECHER erases the entire EEPROM and is NOT UPDI-only, which is
 * exactly why it is not here - one mistaken call would take every
 * setting a product has ever stored. Erasing a whole memory is a
 * provisioning act, and provisioning goes through the programmer.
 */
enum class NvmCommand : uint8_t {
    none = 0x00,               ///< NOCMD - nothing selected
    noop = 0x01,               ///< NOOP - the other way to clear a selection
    flash_write = 0x02,        ///< FLWR
    flash_page_erase = 0x08,   ///< FLPER
    flash_erase2 = 0x09,       ///< FLMPER2
    flash_erase4 = 0x0A,
    flash_erase8 = 0x0B,
    flash_erase16 = 0x0C,
    flash_erase32 = 0x0D,
    eeprom_write = 0x12,       ///< EEWR
    eeprom_erase_write = 0x13, ///< EEERWR
    eeprom_byte_erase = 0x18,  ///< EEBER
    eeprom_erase2 = 0x19,      ///< EEMBER2
    eeprom_erase4 = 0x1A,
    eeprom_erase8 = 0x1B,
    eeprom_erase16 = 0x1C,
    eeprom_erase32 = 0x1D,
    /// Not a command: 0x7F falls in 11.5.1's "Other - Reserved". Kept
    /// named because a caller sometimes needs to ask the controller
    /// what it does with one, and the answer is worth knowing: the
    /// field is drawn as CMD[6:0] but only SIX bits are implemented -
    /// 0x7F reads back as 0x3F - and selecting a reserved code raises
    /// no error by itself. The error comes when a store is attempted
    /// under it (measured; every real command is below 0x40 anyway).
    reserved = 0x7F,
};

/// How many pages one erase command takes down (11.3.2.3.3, table 11-4).
/// The value IS the page count; the address must be aligned to it,
/// because the hardware ignores the low bits of the page number and
/// would erase the aligned block around a misaligned address.
enum class FlashErase : uint8_t {
    page = 1,
    pages2 = 2,
    pages4 = 4,
    pages8 = 8,
    pages16 = 16,
    pages32 = 32,
};

/// How many bytes one EEPROM erase command takes down (11.3.2.3.7,
/// table 11-5). Same alignment rule as FlashErase.
enum class EepromErase : uint8_t {
    byte = 1,
    bytes2 = 2,
    bytes4 = 4,
    bytes8 = 8,
    bytes16 = 16,
    bytes32 = 32,
};

constexpr uint8_t erase_pages(FlashErase e) { return static_cast<uint8_t>(e); }
constexpr uint8_t erase_bytes(EepromErase e) { return static_cast<uint8_t>(e); }

/// A half-open byte range of the physical Flash.
struct FlashRange {
    uint32_t begin;
    uint32_t end;

    constexpr bool empty() const { return end <= begin; }
    constexpr uint32_t size() const { return empty() ? 0u : end - begin; }
    constexpr bool contains(uint32_t b, uint32_t e) const {
        return !empty() && b >= begin && e <= end && e >= b;
    }
};

/**
 * The Flash section geometry as a COMPILE-TIME claim.
 *
 * The real geometry lives in two fuses, so nothing about it is knowable
 * when the code is compiled - and yet "this write must never be able to
 * target BOOT" is exactly the kind of mistake that should not survive
 * to the bench. FlashLayout is how an image states the fuses it is
 * built for: the compile-time verbs (Nvm::erase_at, Nvm::write_word_at)
 * take it and refuse an illegal address with a static_assert, and
 * matches_fuses() cross-checks the claim against the silicon at boot.
 *
 * The parameters are the fuse values, in 512-byte blocks:
 *   boot_blocks  FUSE.BOOTSIZE - 0 means the ENTIRE Flash is BOOT, and
 *                then nothing at all is writable from software;
 *   code_blocks  FUSE.CODESIZE - 0 means APPCODE runs to the end and
 *                there is no APPDATA (table 11-2).
 *
 * Everything below assumes the code is executing from BOOT, which is
 * true of every image linked at address 0: the vector table and the
 * reset entry are the first thing in the Flash, and the BOOT section
 * always starts there.
 */
template <uint8_t boot_blocks, uint8_t code_blocks = 0>
struct FlashLayout {
    static constexpr uint8_t bootsize = boot_blocks;
    static constexpr uint8_t codesize = code_blocks;

    /// First byte ABOVE the BOOT section.
    static constexpr uint32_t boot_end =
        boot_blocks == 0
            ? flash_size
            : (static_cast<uint32_t>(boot_blocks) * flash_section_block >
                       flash_size
                   ? flash_size   // an oversized fuse is ignored (11.3.1.1 note 2)
                   : static_cast<uint32_t>(boot_blocks) * flash_section_block);

    /// First byte above the APPCODE section (== boot_end when there is
    /// no APPCODE at all).
    static constexpr uint32_t appcode_end =
        boot_blocks == 0
            ? flash_size
            : (code_blocks == 0
                   ? flash_size
                   : (code_blocks <= boot_blocks
                          ? boot_end
                          : (static_cast<uint32_t>(code_blocks) *
                                         flash_section_block >
                                     flash_size
                                 ? flash_size
                                 : static_cast<uint32_t>(code_blocks) *
                                       flash_section_block)));

    static constexpr FlashSection section_of(uint32_t addr) {
        return addr < boot_end      ? FlashSection::boot
               : addr < appcode_end ? FlashSection::appcode
                                    : FlashSection::appdata;
    }

    /// Can code running in BOOT write every byte of [b, e)? Table 11-1:
    /// BOOT never, APPCODE and APPDATA yes. Protection bits are runtime
    /// state and are checked separately by the runtime verbs.
    static constexpr bool writable(uint32_t b, uint32_t e) {
        return e > b && e <= flash_size && b >= boot_end;
    }

    /// Does the silicon agree with the claim? Call it once at boot: a
    /// mismatch means the part was programmed with other fuses and every
    /// compile-time refusal above is guarding the wrong boundary.
    static bool matches_fuses() {
        return FUSE.BOOTSIZE == boot_blocks && FUSE.CODESIZE == code_blocks;
    }
};

// =============================================================================
// SIGROW - the factory-programmed row (11.3.1.3)
// =============================================================================

/// The Signature Row: read-only, and the only place the individual chip
/// says who it is. The device ID names the TYPE (three bytes) and the
/// serial number names the SPECIMEN (16 bytes: lot, wafer, coordinates).
/// TEMPSENSE0/1 are the factory calibration of the internal temperature
/// sensor; converting an ADC reading with them is the ADC's business,
/// not this driver's, so they are only exposed here.
struct Sigrow {
    Sigrow() = delete;

    static constexpr uint8_t serial_bytes = 16;

    static uint8_t device_id(uint8_t i) {
        return (&SIGROW.DEVICEID0)[i & 0x3u];
    }
    /// The three ID bytes as one number, high byte first.
    static uint32_t device_id() {
        return (static_cast<uint32_t>(SIGROW.DEVICEID0) << 16) |
               (static_cast<uint32_t>(SIGROW.DEVICEID1) << 8) |
               static_cast<uint32_t>(SIGROW.DEVICEID2);
    }
    static uint8_t serial(uint8_t i) {
        return (&SIGROW.SERNUM0)[i < serial_bytes ? i : 0];
    }
    static uint16_t tempsense0() { return SIGROW.TEMPSENSE0; }
    static uint16_t tempsense1() { return SIGROW.TEMPSENSE1; }
};

// =============================================================================
// NVMCTRL
// =============================================================================

/// The nonvolatile memory controller. One instance, so static verbs -
/// the same shape as Reset and Watchdog in avrdx/reset.hpp.
struct Nvm {
    Nvm() = delete;

    // ---- state -------------------------------------------------------------

    /// STATUS.FBUSY: a Flash programming operation is running. Note that
    /// the CPU is halted for the whole of one, so this can only ever
    /// read true from an interrupt-driven observer or right after a
    /// command that did not start.
    static bool flash_busy() { return (NVMCTRL.STATUS & NVMCTRL_FBUSY_bm) != 0; }
    /// STATUS.EEBUSY: an EEPROM write or erase is running. The CPU keeps
    /// executing meanwhile - this is the flag worth polling.
    static bool eeprom_busy() { return (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) != 0; }
    static bool busy() {
        return (NVMCTRL.STATUS & (NVMCTRL_FBUSY_bm | NVMCTRL_EEBUSY_bm)) != 0;
    }

    /// STATUS.ERROR, the LAST error the controller saw (11.5.3). It is
    /// sticky: it survives until cleared, so read it after a sequence,
    /// not after every store.
    static NvmError error() {
        return static_cast<NvmError>((NVMCTRL.STATUS & NVMCTRL_ERROR_gm) >>
                                     NVMCTRL_ERROR_gp);
    }
    /// Clear ERROR by writing the field to zero (11.5.3 - a plain write,
    /// not a write-one-to-clear). The busy bits are read-only and are
    /// unaffected. STATUS is not CCP-protected.
    static void clear_error() {
        NVMCTRL.STATUS = static_cast<uint8_t>(NVMCTRL.STATUS & ~NVMCTRL_ERROR_gm);
    }

    /// Bounded wait for both busy flags. False when it never cleared - a
    /// stuck controller, not a slow one: the bound is far beyond the
    /// 11.7 ms worst case of table 39-7.
    static bool wait_idle() {
        for (uint32_t i = 0; i < wait_limit; ++i) {
            if (!busy()) {
                return true;
            }
        }
        return false;
    }
    static bool wait_eeprom() {
        for (uint32_t i = 0; i < wait_limit; ++i) {
            if (!eeprom_busy()) {
                return true;
            }
        }
        return false;
    }
    static bool wait_flash() {
        for (uint32_t i = 0; i < wait_limit; ++i) {
            if (!flash_busy()) {
                return true;
            }
        }
        return false;
    }

    /// Release the selected command (11.3.2.3 step 6). Every verb here
    /// does it on its way out; it is public because a caller that armed
    /// a run of writes through the store model closes it explicitly.
    static void clear_command() { write_ctrla(NVMCTRL_CMD_NONE_gc); }

    /// Select a command without carrying it out. The chapter requires a
    /// change from one command to another to pass through NOCMD or NOOP
    /// (11.5.1) - selecting straight over a live one raises a collision
    /// error instead of taking effect - so this clears first.
    static void select(NvmCommand cmd) {
        clear_command();
        write_ctrla(static_cast<uint8_t>(cmd));
    }
    static NvmCommand selected() {
        return static_cast<NvmCommand>(NVMCTRL.CTRLA & NVMCTRL_CMD_gm);
    }

    // ---- CTRLB: mapping and protections ------------------------------------

    /// CTRLB.FLMAP: which 32 KB block of Flash the data-space window
    /// shows (11.5.2). No verb in this driver consults it; the C runtime
    /// sets it for .rodata.
    static uint8_t flmap() {
        return static_cast<uint8_t>((NVMCTRL.CTRLB & NVMCTRL_FLMAP_gm) >>
                                    NVMCTRL_FLMAP_gp);
    }
    /// CTRLB.FLMAPLOCK: FLMAP can no longer be changed. One-way - only a
    /// reset clears it.
    static bool flmap_locked() {
        return (NVMCTRL.CTRLB & NVMCTRL_FLMAPLOCK_bm) != 0;
    }
    /// Move the data-space window. FLMAP[1:0] is the one field of CTRLB
    /// that is NOT under CCP (11.5.2), so this is a plain read-modify-
    /// write. False when the field is locked or the section does not
    /// exist. Reading Flash through the window after this is legal on
    /// both families; WRITING through it is not (DS80000882C 2.7.1) and
    /// no verb here does.
    static bool set_flmap(uint8_t section) {
        if (section > 3 || flmap_locked()) {
            return false;
        }
        NVMCTRL.CTRLB = static_cast<uint8_t>(
            (NVMCTRL.CTRLB & ~NVMCTRL_FLMAP_gm) |
            (static_cast<uint8_t>(section) << NVMCTRL_FLMAP_gp));
        return true;
    }
    /// Freeze FLMAP for the rest of this power-on. CTRLB is CCP-IOREG
    /// protected for every bit except FLMAP itself.
    static void lock_flmap() { set_ctrlb(NVMCTRL_FLMAPLOCK_bm); }

    /// CTRLB.APPCODEWP: refuse all further writes to the APPCODE
    /// section. One-way, cleared only by a reset (11.5.2).
    static void protect_appcode() { set_ctrlb(NVMCTRL_APPCODEWP_bm); }
    static bool appcode_protected() {
        return (NVMCTRL.CTRLB & NVMCTRL_APPCODEWP_bm) != 0;
    }

    /// CTRLB.APPDATAWP: the same for APPDATA. Remember the erratum: the
    /// silicon honours this bit for a single-page erase and IGNORES it
    /// for pages 2..n of a multi-page erase. erase() below does not.
    static void protect_appdata() { set_ctrlb(NVMCTRL_APPDATAWP_bm); }
    static bool appdata_protected() {
        return (NVMCTRL.CTRLB & NVMCTRL_APPDATAWP_bm) != 0;
    }

    /// CTRLB.BOOTRP: make the BOOT section unreadable and unexecutable
    /// from outside it - reads return 0 and instruction fetches return
    /// NOP (11.5.2). Two rules the caller owns, because no guard in
    /// software can enforce either: the bit can only be WRITTEN by code
    /// executing IN the BOOT section, and it takes effect only when
    /// execution LEAVES that section. With an application linked at 0
    /// under a BOOT-sized-BOOT geometry, all code is in BOOT: the write
    /// is legal and nothing ever leaves, so the protection is armed and
    /// dormant until a reset clears it.
    static void protect_boot_read() { set_ctrlb(NVMCTRL_BOOTRP_bm); }
    static bool boot_read_protected() {
        return (NVMCTRL.CTRLB & NVMCTRL_BOOTRP_bm) != 0;
    }

    // ---- the interrupt vector table ----------------------------------------

    /// CPUINT.CTRLA.IVSEL (15.5.1): put the interrupt vectors at the
    /// START of the BOOT section instead of directly after it.
    ///
    /// This is an NVMCTRL concern despite living in another peripheral's
    /// register: the DEFAULT is "vectors at the start of APPCODE", which
    /// is correct only for an image built as a boot loader plus a
    /// separate application. An image linked at 0 - which is every brio
    /// image - puts its vector table at 0, so the moment BOOTSIZE stops
    /// being 0 the hardware starts looking for vectors in empty Flash
    /// and the first interrupt jumps into 0xFFFF. Setting IVSEL is
    /// correct under BOTH geometries (BOOT always starts at 0) and the
    /// build makes it an invariant of every image by doing it in .init3,
    /// before anything can enable an interrupt. This verb is the same
    /// store, callable and readable back at run time.
    ///
    /// When the entire Flash is one BOOT section the bit is ignored by
    /// the hardware (15.5.1) - it still reads back what was written.
    static void vectors_in_boot() {
        const uint8_t v = static_cast<uint8_t>(CPUINT.CTRLA | CPUINT_IVSEL_bm);
        _PROTECTED_WRITE(CPUINT.CTRLA, v);
    }
    static bool vectors_in_boot_armed() {
        return (CPUINT.CTRLA & CPUINT_IVSEL_bm) != 0;
    }

    // ---- section geometry, from the fuses -----------------------------------

    /// First byte above the BOOT section, read from FUSE.BOOTSIZE
    /// (11.3.1.1). BOOTSIZE 0 means the whole Flash is BOOT.
    static uint32_t boot_end() {
        const uint8_t bs = FUSE.BOOTSIZE;
        if (bs == 0) {
            return flash_size;
        }
        const uint32_t e = static_cast<uint32_t>(bs) * flash_section_block;
        return e > flash_size ? flash_size : e;
    }

    /// First byte above the APPCODE section (table 11-2). Equal to
    /// boot_end() when the geometry leaves no APPCODE.
    static uint32_t appcode_end() {
        const uint8_t bs = FUSE.BOOTSIZE;
        if (bs == 0) {
            return flash_size;
        }
        const uint8_t cs = FUSE.CODESIZE;
        if (cs == 0) {
            return flash_size;
        }
        if (cs <= bs) {
            return boot_end();
        }
        const uint32_t e = static_cast<uint32_t>(cs) * flash_section_block;
        return e > flash_size ? flash_size : e;
    }

    static FlashSection section_of(uint32_t addr) {
        if (addr < boot_end()) {
            return FlashSection::boot;
        }
        return addr < appcode_end() ? FlashSection::appcode
                                    : FlashSection::appdata;
    }

    /// Is EVERY byte of [begin, end) writable right now by code running
    /// in BOOT? Section geometry from the fuses, plus the two protection
    /// bits. THIS is the check the multi-page erase erratum makes
    /// necessary: the hardware validates only the first page of a
    /// multi-page range, so a range that starts in an unprotected
    /// section and reaches into a protected one would be erased.
    static bool writable(uint32_t begin, uint32_t end) {
        if (end <= begin || end > flash_size) {
            return false;
        }
        const uint32_t be = boot_end();
        if (begin < be) {
            return false;                // the section we execute from
        }
        const uint32_t ae = appcode_end();
        const bool touches_appcode = begin < ae;
        const bool touches_appdata = end > ae;
        if (touches_appcode && appcode_protected()) {
            return false;
        }
        if (touches_appdata && appdata_protected()) {
            return false;
        }
        return true;
    }

    // ---- Flash: reading -----------------------------------------------------
    //
    // ELPM through a 24-bit address, always. Never `&symbol`: on a
    // 128 KB part a pointer is 16 bits wide and silently loses the top
    // of the memory.

    static uint8_t flash_read(uint32_t addr) {
        return pgm_read_byte_far(addr);
    }
    static uint16_t flash_read_word(uint32_t addr) {
        return pgm_read_word_far(addr);
    }
    static void flash_read(uint32_t addr, uint8_t* dst, uint16_t len) {
        for (uint16_t i = 0; i < len; ++i) {
            dst[i] = pgm_read_byte_far(addr + i);
        }
    }
    /// True when every byte of [begin, end) reads back as the erased
    /// pattern.
    static bool flash_blank(uint32_t begin, uint32_t end) {
        for (uint32_t a = begin; a < end; ++a) {
            if (pgm_read_byte_far(a) != 0xFFu) {
                return false;
            }
        }
        return true;
    }

    // ---- Flash: erasing and writing -----------------------------------------

    /// Erase one page, or an aligned block of 2/4/8/16/32 of them.
    ///
    /// Refuses - without touching the Flash - a range that is not
    /// aligned to its own span, that leaves the device, or that any part
    /// of which is not writable right now. The last of those is the
    /// erratum guard: the hardware would check only the first page.
    ///
    /// The CPU is HALTED for the whole erase (about 10 ms, 11.7 ms
    /// worst case, table 39-7) whatever the span - erasing 32 pages
    /// costs exactly as much time as erasing one (11.3.2.2).
    static bool erase(uint32_t addr, FlashErase span = FlashErase::page) {
        const uint32_t bytes =
            static_cast<uint32_t>(erase_pages(span)) * flash_page_size;
        if (addr % bytes != 0 || !writable(addr, addr + bytes)) {
            return false;
        }
        if (!wait_idle()) {
            return false;
        }
        clear_command();
        write_ctrla(erase_command(span));
        spm_word(addr, 0xFFFFu);          // the value is ignored; the store starts it
        const bool ok = wait_idle();
        clear_command();
        return ok;
    }

    /// The same erase with the SECTION and PROTECTION half of the
    /// validation left out - device bounds and alignment are still
    /// enforced, because those are typos and not decisions.
    ///
    /// This is the raw chapter, erratum included, and it exists for two
    /// reasons that are the same reason: so that what the silicon really
    /// does can be MEASURED rather than believed. Aimed at BOOT it
    /// produces the WRITEPROTECT error the controller is supposed to
    /// produce; aimed at a multi-page range that starts unprotected and
    /// ends protected it erases the protected pages, which is
    /// DS80000915F 2.7.1 / DS80000882C 2.7.2 in the flesh and is exactly
    /// what erase() above refuses to let happen. Applications use
    /// erase(); this is for a test bench and for a bootloader that has
    /// worked out the geometry for itself.
    static bool erase_ignoring_protection(uint32_t addr,
                                          FlashErase span = FlashErase::page) {
        const uint32_t bytes =
            static_cast<uint32_t>(erase_pages(span)) * flash_page_size;
        if (addr % bytes != 0 || addr + bytes > flash_size) {
            return false;
        }
        if (!wait_idle()) {
            return false;
        }
        clear_command();
        write_ctrla(erase_command(span));
        spm_word(addr, 0xFFFFu);
        const bool ok = wait_idle();
        clear_command();
        return ok;
    }

    /// Write one 16-bit word. The Flash is word-organized for writes, so
    /// the address must be even; bit 0 of the address pointer is ignored
    /// by the hardware (11.3.2) and an odd address is a caller's bug,
    /// not a half-word write.
    ///
    /// A write can only clear bits: the target must have been erased.
    /// The CPU is halted for about 70 us (table 39-7).
    static bool write_word(uint32_t addr, uint16_t word) {
        if ((addr & 1u) != 0 || !writable(addr, addr + 2)) {
            return false;
        }
        if (!wait_idle()) {
            return false;
        }
        clear_command();
        write_ctrla(NVMCTRL_CMD_FLWR_gc);
        spm_word(addr, word);
        const bool ok = wait_idle();
        clear_command();
        return ok;
    }

    /// Write a run of words with the command selected once (11.3.2.3.1:
    /// several writes may be done while FLWR is enabled). `len` is a
    /// byte count and must be even, as must the address. The whole range
    /// is validated before the first word goes down.
    static bool write_block(uint32_t addr, const uint8_t* src, uint16_t len) {
        if ((addr & 1u) != 0 || (len & 1u) != 0 || !writable(addr, addr + len)) {
            return false;
        }
        if (!wait_idle()) {
            return false;
        }
        clear_command();
        write_ctrla(NVMCTRL_CMD_FLWR_gc);
        bool ok = true;
        for (uint16_t i = 0; i < len && ok; i += 2) {
            const uint16_t word = static_cast<uint16_t>(
                src[i] | (static_cast<uint16_t>(src[i + 1]) << 8));
            spm_word(addr + i, word);
            ok = wait_idle();
        }
        clear_command();
        return ok;
    }

    // ---- EEPROM -------------------------------------------------------------
    //
    // The EEPROM is memory-mapped in data space and is READ with a plain
    // load - no command, no window, no LPM (11.3.2.3 note: LPM/SPM
    // cannot be used for EEPROM at all). Writing is a command in CTRLA
    // followed by a store to the mapped address; NVMCTRL.DATA and
    // NVMCTRL.ADDR play no part in it - they are observers that report
    // the last value and the last address the controller saw (11.5.6,
    // 11.5.7), not the way an operation is programmed.

    static uint8_t eeprom_read(uint16_t offset) {
        return *reinterpret_cast<volatile uint8_t*>(eeprom_base + offset);
    }
    static void eeprom_read(uint16_t offset, uint8_t* dst, uint16_t len) {
        for (uint16_t i = 0; i < len; ++i) {
            dst[i] = eeprom_read(static_cast<uint16_t>(offset + i));
        }
    }

    /// Write one byte WITHOUT erasing it first (EEWR, 11.3.2.3.4). Only
    /// clears bits, exactly like the Flash: use it when the target is
    /// known to be erased, or when the new value only turns ones into
    /// zeros. Returns as soon as the operation has STARTED - the CPU
    /// keeps running and eeprom_busy() reports the rest.
    static bool eeprom_write(uint16_t offset, uint8_t value) {
        return eeprom_store(offset, value, NVMCTRL_CMD_EEWR_gc);
    }

    /// Erase and write one byte in one operation (EEERWR, 11.3.2.3.5) -
    /// the verb that behaves the way a caller expects "store this byte"
    /// to behave, at the price of the full ~10 ms erase-and-write time
    /// instead of the 70 us of a bare write.
    static bool eeprom_erase_write(uint16_t offset, uint8_t value) {
        return eeprom_store(offset, value, NVMCTRL_CMD_EEERWR_gc);
    }

    /// Erase one byte, or an aligned block of 2/4/8/16/32 of them
    /// (11.3.2.3.6, 11.3.2.3.7). An erased byte reads 0xFF.
    static bool eeprom_erase(uint16_t offset,
                             EepromErase span = EepromErase::byte) {
        const uint16_t bytes = erase_bytes(span);
        if (offset % bytes != 0 ||
            static_cast<uint32_t>(offset) + bytes > eeprom_size) {
            return false;
        }
        return eeprom_store(offset, 0xFFu, erase_command(span));
    }

    /// The store half of the chapter's two-step model, with no command
    /// selected here: put one byte into the EEPROM's mapped address and
    /// let whatever select() armed act on it. False only when the offset
    /// is outside the array.
    ///
    /// With NO command selected the controller does nothing at all and
    /// STATUS.ERROR reports invalid_command - which is the documented
    /// way of asking "is a command still armed?", and the only way to
    /// see that error code at all.
    static bool eeprom_poke(uint16_t offset, uint8_t value) {
        if (offset >= eeprom_size) {
            return false;
        }
        *reinterpret_cast<volatile uint8_t*>(eeprom_base + offset) = value;
        return true;
    }

    /// Store a run of bytes, erase-and-write, with the command selected
    /// once and a bounded wait between bytes. Blocking by construction:
    /// ~10 ms per byte with the CPU free but this call not returning.
    /// The non-blocking way to do the same thing is util/nv_writer.hpp.
    static bool eeprom_write_block(uint16_t offset, const uint8_t* src,
                                   uint16_t len) {
        if (static_cast<uint32_t>(offset) + len > eeprom_size) {
            return false;
        }
        if (!wait_idle()) {
            return false;
        }
        clear_command();
        write_ctrla(NVMCTRL_CMD_EEERWR_gc);
        bool ok = true;
        for (uint16_t i = 0; i < len && ok; ++i) {
            *reinterpret_cast<volatile uint8_t*>(eeprom_base + offset + i) = src[i];
            ok = wait_eeprom();
        }
        clear_command();
        return ok;
    }

    // ---- EEREADY ------------------------------------------------------------

    /// INTFLAGS.EEREADY. A LEVEL flag: it is set continuously while the
    /// EEPROM is not busy, not once per completion (11.5.5).
    static bool eeprom_ready_flag() {
        return (NVMCTRL.INTFLAGS & NVMCTRL_EEREADY_bm) != 0;
    }
    static void clear_eeprom_ready_flag() {
        NVMCTRL.INTFLAGS = NVMCTRL_EEREADY_bm;
    }
    /// INTCTRL.EEREADY. Two rules from 11.5.4, both of them consequences
    /// of the flag being a level: the interrupt must NOT be enabled
    /// before the write that it is meant to report has been started (the
    /// flag is already standing), and the handler MUST disable it again
    /// or the vector re-enters forever.
    static void enable_eeprom_ready_interrupt(bool on) {
        NVMCTRL.INTCTRL = on ? NVMCTRL_EEREADY_bm : 0u;
    }

    /// The NVMCTRL interrupt's handler body: disable the level interrupt
    /// that just fired, which is the one thing 11.5.4 requires and the
    /// one thing that cannot wait for the AO to be dispatched. The app
    /// binds the vector and posts whatever its writer expects:
    ///
    ///     ISR(NVMCTRL_EE_vect) { Nvm::eeready(); post<Writer>(NvReady{}); }
    [[gnu::always_inline]] static void eeready() {
        NVMCTRL.INTCTRL = 0;
    }

    // ---- USERROW ------------------------------------------------------------
    //
    // 11.3.2.2: "The User Row is erased/written as a normal Flash" - the
    // FLASH commands, not the EEPROM ones - but the row is memory-mapped
    // in data space, so the store that carries out the command is a
    // plain ST and the granularity is one byte (table 11-3 note 1). An
    // erase takes the WHOLE 32-byte row down at once, there being only
    // one page of it.
    //
    // This is provisioning storage, and on a brio bench it holds the
    // board's identity label (avrdx/userrow.hpp). A verb here can wipe
    // that: userrow_erase() is the only way to turn a zero back into a
    // one, and it does it to all 32 bytes.

    static uint8_t userrow_read(uint8_t offset) {
        return *reinterpret_cast<volatile uint8_t*>(userrow_base + offset);
    }
    static void userrow_read(uint8_t offset, uint8_t* dst, uint8_t len) {
        for (uint8_t i = 0; i < len; ++i) {
            dst[i] = userrow_read(static_cast<uint8_t>(offset + i));
        }
    }

    /// Write one byte of the row (FLWR + a store). Bits can only be
    /// cleared; the byte must read 0xFF, or the value must only turn
    /// ones into zeros.
    static bool userrow_write(uint8_t offset, uint8_t value) {
        if (offset >= userrow_size) {
            return false;
        }
        return userrow_store(offset, value, NVMCTRL_CMD_FLWR_gc);
    }

    static bool userrow_write_block(uint8_t offset, const uint8_t* src,
                                    uint8_t len) {
        if (static_cast<uint16_t>(offset) + len > userrow_size) {
            return false;
        }
        if (!wait_idle()) {
            return false;
        }
        clear_command();
        write_ctrla(NVMCTRL_CMD_FLWR_gc);
        bool ok = true;
        for (uint8_t i = 0; i < len && ok; ++i) {
            *reinterpret_cast<volatile uint8_t*>(userrow_base + offset + i) = src[i];
            ok = wait_idle();
        }
        clear_command();
        return ok;
    }

    /// Erase ALL 32 bytes of the User Row (FLPER + a store). There is no
    /// smaller unit: the row is one page.
    static bool userrow_erase() {
        return userrow_store(0, 0xFFu, NVMCTRL_CMD_FLPER_gc);
    }

    // ---- the scratch region -------------------------------------------------

    /**
     * The Flash an image is free to use as scratch: everything between
     * the end of the executable image and the start of its read-only
     * data, clipped to start above the BOOT section.
     *
     * The shape of the hole is a fact of this toolchain. gcc 16 places
     * .rodata in a 32 KB Flash section reached through the data-space
     * window (the default is section 3, 96 KB in) and leaves the code
     * and the .data initializers low, so a 128 KB part with a 23 KB
     * image has a large contiguous hole in the MIDDLE, not a small
     * remainder at the top. That hole is where the scratch belongs: its
     * upper bound is a linker constant that does not move, and its
     * lower bound is the BOOT boundary, which is a hardware one - so the
     * region is the same set of pages on every run of a given image, and
     * a wear budget stated in pages means something.
     *
     * The bounds are EXACT, with no safety margin subtracted: both are
     * facts, not estimates - the top one is where .rodata's load address
     * begins and the bottom one is a hardware section boundary - and a
     * margin would only cost the region its alignment. What that
     * alignment buys is that the whole span of the largest erase command
     * (32 pages, 16 KB) has somewhere to land inside the region.
     *
     * Empty (begin == end) when the geometry leaves no such hole - which
     * is the case for the shipping fuse default, BOOTSIZE = 0, where the
     * whole Flash is BOOT and nothing at all is writable from software.
     */
    static FlashRange scratch_region() {
        const uint32_t image = align_up(image_low_end(), flash_page_size);
        const uint32_t be = boot_end();
        const uint32_t lo = image > be ? image : be;
        const uint32_t hi = align_down(rodata_load_start(), flash_page_size);
        return hi > lo ? FlashRange{lo, hi} : FlashRange{lo, lo};
    }

    /// LMA just past the low half of the image: code, vectors and the
    /// initializers of .data.
    static uint32_t image_low_end() {
        return pgm_get_far_address(__data_load_end);
    }
    /// LMA where the image's read-only data begins.
    static uint32_t rodata_load_start() {
        return pgm_get_far_address(__rodata_load_start);
    }
    /// LMA just past it.
    static uint32_t rodata_load_end() {
        return pgm_get_far_address(__rodata_load_end);
    }

    // ---- compile-time verbs -------------------------------------------------
    //
    // Same operations, with the address and the span as template
    // parameters and the image's declared FlashLayout as the authority:
    // what the geometry forbids does not compile.

    template <typename Layout, uint32_t addr, FlashErase span = FlashErase::page>
    static bool erase_at() {
        constexpr uint32_t bytes =
            static_cast<uint32_t>(erase_pages(span)) * flash_page_size;
        static_assert(addr < flash_size,
                      "the address is beyond this device's Flash");
        static_assert(addr % bytes == 0,
                      "a Flash erase starts at a multiple of its own span: the "
                      "hardware ignores the low page bits and would erase the "
                      "aligned block around this address");
        static_assert(Layout::writable(addr, addr + bytes),
                      "this range is not writable under the declared Flash "
                      "layout - the BOOT section cannot be written by the code "
                      "executing from it");
        return erase(addr, span);
    }

    template <typename Layout, uint32_t addr>
    static bool write_word_at(uint16_t word) {
        static_assert(addr < flash_size,
                      "the address is beyond this device's Flash");
        static_assert((addr & 1u) == 0,
                      "the Flash is written one WORD at a time: the address "
                      "must be even");
        static_assert(Layout::writable(addr, addr + 2),
                      "this address is not writable under the declared Flash "
                      "layout");
        return write_word(addr, word);
    }

    template <uint16_t offset, EepromErase span = EepromErase::byte>
    static bool eeprom_erase_at() {
        static_assert(offset < eeprom_size,
                      "the offset is beyond this device's EEPROM");
        static_assert(offset % erase_bytes(span) == 0,
                      "an EEPROM erase starts at a multiple of its own span");
        static_assert(offset + erase_bytes(span) <= eeprom_size,
                      "the erase span runs past the end of the EEPROM");
        return eeprom_erase(offset, span);
    }

    template <uint8_t offset>
    static bool userrow_write_at(uint8_t value) {
        static_assert(offset < userrow_size,
                      "the offset is beyond the 32-byte User Row");
        return userrow_write(offset, value);
    }

private:
    /// Generous by design: the longest operation in table 39-7 is 11.7
    /// ms and this loop is a few cycles, so the bound exists only so a
    /// controller that never answers cannot hang the caller.
    static constexpr uint32_t wait_limit = 4'000'000u;

    /// CTRLA is CCP-protected with the SPM key (table 11-7).
    static void write_ctrla(uint8_t cmd) {
        _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, cmd);
    }

    /// CTRLB is CCP-protected with the IOREG key (table 11-7), FLMAP
    /// excepted. Every bit this touches is one-way, so the read-modify-
    /// write can never lose one.
    static void set_ctrlb(uint8_t bits) {
        const uint8_t v = static_cast<uint8_t>(NVMCTRL.CTRLB | bits);
        _PROTECTED_WRITE(NVMCTRL.CTRLB, v);
    }

    static constexpr uint8_t erase_command(FlashErase span) {
        switch (span) {
            case FlashErase::pages2: return NVMCTRL_CMD_FLMPER2_gc;
            case FlashErase::pages4: return NVMCTRL_CMD_FLMPER4_gc;
            case FlashErase::pages8: return NVMCTRL_CMD_FLMPER8_gc;
            case FlashErase::pages16: return NVMCTRL_CMD_FLMPER16_gc;
            case FlashErase::pages32: return NVMCTRL_CMD_FLMPER32_gc;
            case FlashErase::page: break;
        }
        return NVMCTRL_CMD_FLPER_gc;
    }

    static constexpr uint8_t erase_command(EepromErase span) {
        switch (span) {
            case EepromErase::bytes2: return NVMCTRL_CMD_EEMBER2_gc;
            case EepromErase::bytes4: return NVMCTRL_CMD_EEMBER4_gc;
            case EepromErase::bytes8: return NVMCTRL_CMD_EEMBER8_gc;
            case EepromErase::bytes16: return NVMCTRL_CMD_EEMBER16_gc;
            case EepromErase::bytes32: return NVMCTRL_CMD_EEMBER32_gc;
            case EepromErase::byte: break;
        }
        return NVMCTRL_CMD_EEBER_gc;
    }

    static bool eeprom_store(uint16_t offset, uint8_t value, uint8_t cmd) {
        if (offset >= eeprom_size) {
            return false;
        }
        if (!wait_idle()) {
            return false;
        }
        clear_command();
        write_ctrla(cmd);
        *reinterpret_cast<volatile uint8_t*>(eeprom_base + offset) = value;
        return true;
    }

    static bool userrow_store(uint8_t offset, uint8_t value, uint8_t cmd) {
        if (!wait_idle()) {
            return false;
        }
        clear_command();
        write_ctrla(cmd);
        *reinterpret_cast<volatile uint8_t*>(userrow_base + offset) = value;
        const bool ok = wait_idle();
        clear_command();
        return ok;
    }

    /// One word into the Flash array through SPM, with the 24-bit
    /// address in RAMPZ:Z and the data in r0:r1. r1 is the compiler's
    /// zero register, so it is cleared again on the way out; RAMPZ is
    /// saved and restored because the caller's code (and any interrupt
    /// that is not masked here) uses it for its own far reads.
    [[gnu::always_inline]] static void spm_word(uint32_t addr, uint16_t word) {
        const uint8_t hh = static_cast<uint8_t>(addr >> 16);
        const uint16_t z = static_cast<uint16_t>(addr);
        uint8_t saved;
        __asm__ __volatile__(
            "in   %[sv], %[rampz]   \n\t"
            "out  %[rampz], %[hh]   \n\t"
            "movw r0, %[w]          \n\t"
            "spm                    \n\t"
            "clr  r1                \n\t"
            "out  %[rampz], %[sv]   \n\t"
            : [sv] "=&d"(saved)
            : [rampz] "I"(_SFR_IO_ADDR(RAMPZ)), [hh] "r"(hh), [w] "r"(word),
              "z"(z)
            : "r0", "memory");
    }

    static constexpr uint32_t align_up(uint32_t v, uint32_t to) {
        return (v + to - 1u) / to * to;
    }
    static constexpr uint32_t align_down(uint32_t v, uint32_t to) {
        return v / to * to;
    }
};

/**
 * The EEPROM as a util/nv_record.hpp store: the backend that lets the
 * target-independent record layout, the writer AO and the persistent
 * panic record run on this silicon (and on the host, against an array,
 * in the same source).
 *
 * write() is erase-and-write, the honest "store this byte" of the two
 * EEPROM write commands, and it RETURNS BEFORE THE BYTE IS DOWN - the
 * ~10 ms belong to the store, not to the caller. ready()/wait_ready()
 * and the ready interrupt are how the two callers above cope with that,
 * each in its own way.
 */
struct EepromStore {
    EepromStore() = delete;

    static constexpr uint16_t size() { return eeprom_size; }
    static uint8_t read(uint16_t addr) { return Nvm::eeprom_read(addr); }
    static bool write(uint16_t addr, uint8_t value) {
        return Nvm::eeprom_erase_write(addr, value);
    }
    static bool ready() { return !Nvm::eeprom_busy(); }
    static bool wait_ready() { return Nvm::wait_eeprom(); }
    static void finish() { Nvm::clear_command(); }

    static void arm_ready_interrupt(bool on) {
        Nvm::enable_eeprom_ready_interrupt(on);
    }
    static bool ready_flag() { return Nvm::eeprom_ready_flag(); }
    static void clear_ready_flag() { Nvm::clear_eeprom_ready_flag(); }
};

static_assert(NvStore<EepromStore>);
static_assert(NvPacedStore<EepromStore>);

} // namespace brio
