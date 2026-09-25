/*
 * nvm.hpp
 *
 * The flash memory and the user option bytes (RM ch. 32) with the
 * electronic signature beside them (ch. 31): `Flash`, the program and
 * erase engine this stratum's storage media are built on, and the one
 * place that writes FLASH_CTLR.
 *
 * TWO PROGRAMMING METHODS AND FOUR ERASE GRAINS. 32.2.1 offers the
 * chapter's own pair: the STANDARD method writes one HALF-WORD at a
 * time behind PG and erases 4 KB at a time behind PER, and the FAST
 * method writes a whole 256-byte PAGE behind FTPG and erases 256 bytes
 * behind FTER, 32 KB behind BER32 or the whole user array behind MER.
 * All of them are here, because the chapter has them and because the
 * two grains answer different questions: the half-word is what a
 * program that changes two bytes wants, the page is what a medium
 * wants. The FlashMedia in nvm_flash.hpp uses the fast pair alone.
 *
 * THE ERASED PATTERN IS NOT 0xFF. 32.5.4 and 32.5.7 say it four times:
 * "after erasing is successful, word read - 0xe339e339, half word read
 * - 0xe339, even address byte read - 0x39, odd address read 0xe3". So
 * an erased cell of this array reads back 0xE339E339 and not the all-
 * ones every other flash in this repository leaves, which is a fact a
 * reader of the array must hold and a structure ABOVE the medium must
 * be built for - see nvm_flash.hpp, where it is one of the reasons
 * nothing is built above it. `erased_word` is that pattern, MEASURED on
 * an erased page and on the third word of this die's own signature; the
 * sister family's chapter promises 0xFF for the same verbs, so this is
 * not a WCH habit but this silicon's.
 *
 * AND THE CELLS DO NOT BEHAVE LIKE BITS THAT ONLY FALL. Measured
 * consequences of that pattern, all of which a structure above the
 * medium would otherwise get wrong. A HALF-WORD CELL OF THE WINDOW
 * TAKES PASS AFTER PASS between erases - five in a row read back
 * EXACTLY what was written, 0x0000 then 0xA5A5 among them, on the
 * CH32V203C8 and the CH32V303VC alike, so bits come back as well as go
 * away, where every other flash in this project can only clear them. A
 * half-word cell of the TAIL does not: programmed 0x9659 and then
 * 0x0F0F with no erase between, it read 0x0E0F - neither value - on the
 * CH32V303VC, so what the window's cells do is the window's and not the
 * array's. And a PAGE PROGRAMMED TWICE with no erase between reports no
 * error and ends up holding NEITHER pattern and not their bitwise AND
 * either, so a torn write cannot be repaired by writing again: a medium
 * erases first, which is what `program()` in nvm_flash.hpp makes its
 * caller do.
 *
 * THREE LOCKS, AND A WRONG KEY LOCKS UNTIL RESET. The FPEC lock (KEYR,
 * CTLR.LOCK) guards CTLR and every standard operation; the FAST lock
 * (MODEKEYR, CTLR.FLOCK) guards the fast ones on top of it; the OPTION
 * lock (OBKEYR, CTLR.OBWRE) guards the option bytes, which this driver
 * only ever reads. Each wants its own key pair, KEY1 then KEY2, written
 * in that order and consecutively; 32.5.2 and 32.5.5 both say that a
 * wrong sequence locks the block "until the next system reset", and
 * 32.5.2 adds that it also "generates a bus error". MEASURED: the lock
 * does hold - a correct pair written afterwards is ignored and every
 * write verb answers `refused` until the chip is reset - and NO BUS
 * ERROR IS RAISED, the store being taken in silence. So a mistyped key
 * is not a fault a program can catch, only a door that stays shut.
 * Nothing here writes a wrong key.
 *
 * THE RATE IS PART OF THE CONTRACT, AND THE ENGINE NEVER HALVES THE
 * TREE IN SILENCE. 32.1 is explicit: the flash access clock must not
 * exceed 60 MHz (the datasheet's Fprog, table 4-17, which covers read,
 * program and erase alike), and FLASH_CTLR.SCKMOD chooses whether that
 * clock is SYSCLK or SYSCLK/2 - half of it out of reset. Above
 * `flash_safe_sysclk_hz` (120 MHz = twice 60, clock.hpp) an erase or a
 * program therefore wants HCLK halved around it, "and then restored
 * after the FLASH operation is completed". This driver will not do that
 * behind a program's back: HALVING HCLK MOVES EVERY PERIPHERAL'S
 * DIVISOR, which is the caller's business and not a storage driver's.
 * So every erase and every program takes the CLOCK as its first
 * argument and refuses the rate instead - at COMPILE TIME under a
 * static Clock (`Clock::flash_needs_halving`), and at RUN TIME with a
 * code (`refused_rate`) under a DynamicClock, whose rate is a fact of
 * the moment. A program that wants to write at 144 MHz steps down
 * through its DynamicClock first and steps back afterwards.
 *
 * THE ADDRESS THE ENGINE TAKES is the 0x0800 0000 alias of the array,
 * which is what the chapter's own examples write through (32.5.6 step
 * 5); the image runs from the alias at zero and the media above number
 * their bytes from zero too, so `alias_of()` is the one translation and
 * it is applied here and nowhere else.
 *
 * THE ENHANCED READ MODE IS A TRAP THE VERBS COVER - WHERE IT ENGAGES.
 * 32.3's mode makes reads faster and its note 1 says that any erase or
 * program attempted while it stands WILL FAIL, so every erase and
 * program verb here refuses while FLASH_STATR.EHMODS reads back set
 * rather than starting an operation the chapter promises will not work.
 * MEASURED on the CH32V203C8 and on the CH32V303VC: EHMOD takes the
 * write and reads back, and EHMODS NEVER FOLLOWS IT - on a part whose
 * option byte carries the flash-and-RAM split 32.3 speaks of (table
 * 32-4 gives it to the CH32V203RB and to the CH32V303RC and VC) as on
 * one whose byte does not. `enhanced_read(true)` therefore answers
 * false there, after a bounded wait, and the refusal it guards is a
 * path no program on either part can reach.
 *
 * THE OPTION BYTES ARE READ ONLY HERE, and that is a decision rather
 * than an omission. Releasing read protection "will first cause the
 * system to automatically perform a whole chip erasure" (32.6.4) and
 * every other option byte is provisioning - what the part does at the
 * next boot, not what a program does now - so OBPG, OBER and the
 * OBKEYR unlock have no verb: `FlashOptions` decodes what the hardware
 * LOADED (FLASH_OBR and FLASH_WPR) and `FlashOptionArea` reads the
 * eight bytes in flash beside their inverses. RDP IS NEVER WRITTEN BY
 * ANY VERB IN THIS FILE.
 *
 * WHAT IS NOT HERE: the system boot loader at 0x1FFF8000 (28 KB of
 * WCH's own, which this project does not call), and the "vendor
 * configuration word" 32.1's note names and locks before delivery.
 *
 * THE ARRAY IS A WINDOW AND A TAIL, and three numbers of the part
 * table say where each ends. Both datasheets count a part's "flash
 * bytes" as the ZERO-WAIT run area and give the die more above it:
 * "for the V203 series, non-zero-wait area is (224K-R0WAIT)" (the
 * CH32V203 datasheet's note 1 to table 2-1), and "the product of 256K
 * FLASH+64K SRAM supports the non-zero waiting area of (480K-R0WAIT)
 * bytes" (the CH32V303 datasheet's note 1 to table 2-1-1), which gives
 * the tail to the CH32V303RC and VC and none to the CH32V303CB and RB.
 * So `window_bytes` is `device::flash_bytes` - what every linker script
 * and the medium reckon with - `tail_bytes` is the non-zero-wait area
 * above it and `array_bytes` the die's whole code flash; `tail_base`
 * and `tail_end` bound the tail, counted from zero like every address
 * here. THE WINDOW IS READ THROUGH THE ALIAS the image runs at (`read()`,
 * `erased()`), and THE TAIL THROUGH THE ARRAY'S OWN ADDRESS
 * (`read_tail()`): the memory map gives 0x0800 0000 the whole code
 * flash, "includes 0 wait and non-0 waiting areas", and says of the
 * alias at zero only that it follows the BOOT pins. WHAT THE TAIL COSTS
 * is the non-zero wait, MEASURED on the CH32V303VC: a word read out of
 * it takes 20 core cycles at 96 MHz against the window's 5, the same 20
 * at 48 MHz with SCKMOD at half and 12 with it whole - SCKMOD moves the
 * tail's price and not the window's, which is 5 at every setting.
 *
 * THE TAIL TAKES BOTH METHODS. 32.2.1's note reads "Fast programming
 * related functions can only be placed in the zero-wait area FLASH"
 * (V2.5: "FLASH erase related functions"), and could be read as a
 * statement about the TARGET those operations write. MEASURED on the
 * CH32V303VC, it is not: a fast page program into the tail's last page
 * landed there byte for byte, a fast page erase erased it and a 32 KB
 * erase erased a block of the tail, each with BSY down and EOP up - so
 * the note is about where the CODE running them sits, which an image
 * linked into the window satisfies by construction, and every write
 * verb here takes a tail address as it takes a window one.
 *
 * A TAIL READ WANTS THE RATE CONTRACT TOO. 32.1's note 2 lists "the
 * non-zero-wait area FLASH" among the flash operations that want HCLK
 * halved above 120 MHz, and the tail is read out of the array at the
 * access clock SCKMOD sets - 72 MHz at 144, over the 60 the chapter
 * allows. The window is not: the core executes out of it at the part's
 * full 144 MHz with SCKMOD at its reset value, which is WCH's own
 * configuration and every image here. So `read_tail()` takes the clock
 * as the write verbs do and refuses the same rates the same two ways.
 * A RATING AND NOT A MEASUREMENT: on the CH32V303VC a page of the tail
 * read by hand at 144 MHz came back exact, twice, which says what one
 * die did outside the rating and nothing about the rating itself.
 *
 * THE SPLIT IS AN OPTION BYTE, DECODED AND NEVER WRITTEN. USER[7:5]
 * moves the line between the window and the SRAM on the parts table
 * 32-4 names (`FlashSplitTable`, one per part in parts/), and
 * `flash_split_of()` is its decode - five combinations on the CH32V303RC
 * and VC, three on the CH32V203RB. FLASH_OBR carries the loaded USER
 * byte WHOLE at bits [9:2], so the split is OBR[9:7]: 32.4.6's bit
 * column prints [9:8] beside three-bit codes, and what settles it is
 * the register itself - OBR 0x000000FC against a USER byte of 0x3F on
 * a CH32V203C8, 0x0000027C against 0x9F on a CH32V303VC, whose code 4
 * decodes to the 256 KB of window and 64 KB of SRAM its part table and
 * linker script state. A program whose split is not the part table's
 * has a linker script that is not its part's either;
 * `split_matches_part()` is the check it can make at boot.
 *
 * ESIG_FLACAP IS NOT THE WINDOW IN FORCE. It reads 288 on a CH32V303VC
 * whose option byte selects 256 KB of window: the LARGEST window table
 * 32-4 offers that part, and on a part with no split the window itself
 * (64 on the CH32V203C8) - both measured. `flash_window_max_bytes`
 * states it, so a program comparing the signature with its build
 * compares the right two numbers.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <optional>
#include <span>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"

namespace brio {

// =============================================================================
// The register bits (RM 32.4)
// =============================================================================
// FlashRegs, the two keys and the two base addresses are device.hpp's,
// beside the other blocks whose map more than one chapter reads. The
// BITS are this chapter's and live here.

/// FLASH_STATR (32.4.3). EOP and WRPRTERR are write-one-to-clear; the
/// rest are read-only. There is no PGERR bit on this family, although
/// ERRIE's own description names one.
inline constexpr uint32_t flash_bsy      = 1UL << 0;   ///< an operation is running
inline constexpr uint32_t flash_wrbsy    = 1UL << 1;   ///< a fast-program word is being taken
inline constexpr uint32_t flash_wrprterr = 1UL << 4;   ///< a protected address was written
inline constexpr uint32_t flash_eop      = 1UL << 5;   ///< the last operation finished
inline constexpr uint32_t flash_ehmods   = 1UL << 7;   ///< the enhanced read mode is in force
/// The two flags a store of ones clears, and the one error the chapter
/// leaves behind.
inline constexpr uint32_t flash_statr_w1c = flash_eop | flash_wrprterr;
inline constexpr uint32_t flash_errors    = flash_wrprterr;

/// FLASH_CTLR (32.4.4).
inline constexpr uint32_t flash_pg       = 1UL << 0;    ///< standard programming (half-words)
inline constexpr uint32_t flash_per      = 1UL << 1;    ///< standard erase (4 KB)
inline constexpr uint32_t flash_mer      = 1UL << 2;    ///< the whole user array
inline constexpr uint32_t flash_obpg     = 1UL << 4;    ///< option-byte programming (not a verb here)
inline constexpr uint32_t flash_ober     = 1UL << 5;    ///< option-byte erase (not a verb here)
inline constexpr uint32_t flash_strt     = 1UL << 6;    ///< start, self-clearing
inline constexpr uint32_t flash_lock     = 1UL << 7;    ///< write-one to re-lock the FPEC
inline constexpr uint32_t flash_obwre    = 1UL << 9;    ///< set by the option key, cleared by software
inline constexpr uint32_t flash_errie    = 1UL << 10;
inline constexpr uint32_t flash_eopie    = 1UL << 12;
inline constexpr uint32_t flash_flock    = 1UL << 15;   ///< write-one to re-lock the fast mode
inline constexpr uint32_t flash_ftpg     = 1UL << 16;   ///< fast page programming (256 bytes)
inline constexpr uint32_t flash_fter     = 1UL << 17;   ///< fast page erase (256 bytes)
inline constexpr uint32_t flash_ber32    = 1UL << 18;   ///< fast block erase (32 KB)
inline constexpr uint32_t flash_pgstrt   = 1UL << 21;   ///< start a fast page program, self-clearing
inline constexpr uint32_t flash_rsenact  = 1UL << 22;   ///< write-one: leave the enhanced read mode
inline constexpr uint32_t flash_ehmod    = 1UL << 24;   ///< the enhanced read mode
inline constexpr uint32_t flash_sckmod   = 1UL << 25;   ///< 1: the array is read at SYSCLK, 0: at half

/// The mode bits a read-modify-write of CTLR must not carry over, and
/// the write-one bits it must not raise by accident: LOCK and FLOCK
/// re-lock the engine, STRT and PGSTRT start an operation, RSENACT
/// leaves a mode. Every store this file makes masks them out.
inline constexpr uint32_t flash_ctlr_write_once =
    flash_lock | flash_flock | flash_strt | flash_pgstrt | flash_rsenact;

/// FLASH_OBR (32.4.6), as the hardware LOADED it at the last system
/// reset. Every bit is read-only.
inline constexpr uint32_t flash_obr_oberr        = 1UL << 0;
inline constexpr uint32_t flash_obr_rdprt        = 1UL << 1;
inline constexpr uint32_t flash_obr_iwdg_sw      = 1UL << 2;
inline constexpr uint32_t flash_obr_stop_rst     = 1UL << 3;
inline constexpr uint32_t flash_obr_standby_rst  = 1UL << 4;
/// The loaded USER byte sits in FLASH_OBR WHOLE, bits [9:2] - the three
/// bits above are USER[0..2], which is where the watchdog's and the two
/// reset bits come from - so the memory split, USER[7:5], is OBR[9:7].
/// 32.4.6's bit column prints the field as [9:8] beside codes three bits
/// wide; the register says otherwise (the file header).
inline constexpr uint32_t flash_obr_user_shift   = 2;
inline constexpr uint32_t flash_obr_split_shift  = 7;
inline constexpr uint32_t flash_obr_split_mask   = 0x7UL << flash_obr_split_shift;

/// Where USER[7:5] sits in the USER option byte itself (table 32-4).
inline constexpr uint8_t flash_user_split_shift = 5;

/// The flash access clock's ceiling (datasheet table 4-17's Fprog, whose
/// note says it covers "read operation, program operation and erase
/// operation" and that "the clock is from HCLK"). SCKMOD decides whether
/// that clock is HCLK or half of it, which is why flash_safe_sysclk_hz
/// (clock.hpp) is twice this number.
inline constexpr uint32_t flash_access_max_hz = 60'000'000UL;

/// The value the user option byte RDPR must hold for the array to be
/// readable (32.6, table 32-4). Named because a decoder should say what
/// it compares against, never because anything here writes it.
inline constexpr uint8_t flash_rdpr_unprotected = 0xA5;

// =============================================================================
// The memory split (RM 32.4.6, table 32-4)
// =============================================================================

/// One combination of USER[7:5]: how much of the code flash is the
/// zero-wait window, and how much SRAM is left beside it - the two
/// numbers the chapter prints for each code.
struct FlashSplit {
    uint16_t code_kbytes;   ///< the zero-wait window
    uint16_t ram_kbytes;    ///< the SRAM beside it

    constexpr bool operator==(const FlashSplit&) const = default;
};

/**
 * USER[7:5] as the table a part reads it by decodes it - nothing on a
 * part with no split. Table 32-4, both halves:
 *
 *   the CH32V303RC and VC (note 1):   00x 192+128, 01x 224+96, 10x 256+64,
 *                                     110 128+192, 111 288+32
 *   the CH32V20x_D8 (the CH32V203RB): 00x 128+64, 01x 144+48, 1xx 160+32
 *
 * 110 on the first table is 128+192 only "where the penultimate sixth
 * digit of the lot number is not zero" (note 2); what the other lots do
 * with that code the chapter does not say, and this decode answers the
 * table's row.
 */
constexpr std::optional<FlashSplit> flash_split_of(FlashSplitTable table, uint8_t code) {
    const uint8_t c = static_cast<uint8_t>(code & 0x7u);
    switch (table) {
        case FlashSplitTable::code_128k_ram_64k:
            if ((c & 0x4u) != 0u) {
                return FlashSplit{160, 32};
            }
            return (c & 0x2u) != 0u ? FlashSplit{144, 48} : FlashSplit{128, 64};
        case FlashSplitTable::code_256k_ram_64k:
            switch (c >> 1) {
                case 0: return FlashSplit{192, 128};
                case 1: return FlashSplit{224, 96};
                case 2: return FlashSplit{256, 64};
                default: return c == 0x6u ? FlashSplit{128, 192} : FlashSplit{288, 32};
            }
        case FlashSplitTable::none:
            break;
    }
    return std::nullopt;
}

/// The largest window a table can select, in kilobytes: 160 and 288 for
/// the two tables, zero for a part with no split.
constexpr uint16_t flash_split_max_kbytes(FlashSplitTable table) {
    return table == FlashSplitTable::code_128k_ram_64k   ? 160u
           : table == FlashSplitTable::code_256k_ram_64k ? 288u
                                                         : 0u;
}

/// WHAT ESIG_FLACAP READS on this part: the largest zero-wait window its
/// option byte can select, and on a part with no split the window
/// itself. Measured on the CH32V303VC (288 while the split in force is
/// 256) and on the CH32V203C8 (64); the CH32V203RB's 160 is the table's
/// and not a measurement.
inline constexpr uint32_t flash_window_max_bytes =
    flash_split_max_kbytes(device::flash_split_table) != 0u
        ? static_cast<uint32_t>(flash_split_max_kbytes(device::flash_split_table)) * 1024UL
        : device::flash_bytes;

// =============================================================================
// The option bytes, decoded and read-only
// =============================================================================

/**
 * What the hardware loaded out of the option byte area at the last
 * system reset: FLASH_OBR's bits and FLASH_WPR's thirty-two, which
 * together are what the silicon is ACTING ON. (What is written in flash
 * is `FlashOptionArea` below; the two differ for exactly as long as it
 * takes a reset to reload them.)
 *
 * The two reset-related bits are stated the way a reader thinks about
 * them and not the way the register spells them: 32.4.6 gives 0 for
 * "system will be reset when entering Stop mode", so `resets_on_stop`
 * is the INVERSE of STOP_RST. The watchdog and power chapters cite
 * these three.
 */
struct FlashOptions {
    bool error;                 ///< OBERR: a byte did not match its inverse at load time
    bool read_protected;        ///< RDPRT
    bool iwdg_software;         ///< IWDG_SW: the independent watchdog waits for software
    bool resets_on_stop;        ///< the chip is reset when it enters Stop
    bool resets_on_standby;     ///< the chip is reset when it enters Standby
    uint32_t write_protection;  ///< FLASH_WPR as loaded: a CLEAR bit is a PROTECTED 4 KB sector
    uint8_t split_code;         ///< USER[7:5] as loaded, OBR[9:7]: the memory split's code

    static FlashOptions read() {
        const uint32_t obr = flash_ctl()->OBR;
        return FlashOptions{
            (obr & flash_obr_oberr) != 0u,
            (obr & flash_obr_rdprt) != 0u,
            (obr & flash_obr_iwdg_sw) != 0u,
            (obr & flash_obr_stop_rst) == 0u,
            (obr & flash_obr_standby_rst) == 0u,
            flash_ctl()->WPR,
            static_cast<uint8_t>((obr & flash_obr_split_mask) >> flash_obr_split_shift),
        };
    }

    /// Is the 4 KB sector at index `n` write-protected? A bit of WPR is
    /// one sector; the last bit covers sectors 31 to 127, which is why a
    /// index above 30 asks about bit 31 (32.6's WRPR3 note) - the whole
    /// tail, on a part whose window is 128 KB or more.
    constexpr bool sector_protected(uint32_t n) const {
        const uint32_t bit = n > 30u ? 31u : n;
        return (write_protection & (1UL << bit)) == 0u;
    }

    /// The same question asked with an address of the array, counted
    /// from zero as the image is linked.
    constexpr bool address_protected(uint32_t addr) const {
        return sector_protected(addr / device::flash_protect_bytes);
    }

    /// The memory split the loaded code names on THIS part, or nothing
    /// where the part has no split (table 32-4).
    constexpr std::optional<FlashSplit> split() const {
        return flash_split_of(device::flash_split_table, split_code);
    }

    /// Does the window the option byte selected agree with the one the
    /// part table - and so the linker script - states? True on a part
    /// with no split, where the question does not arise.
    constexpr bool split_matches_part() const {
        const std::optional<FlashSplit> s = split();
        return !s.has_value() ||
               (static_cast<uint32_t>(s->code_kbytes) * 1024UL == device::flash_bytes &&
                static_cast<uint32_t>(s->ram_kbytes) * 1024UL == device::sram_bytes);
    }
};

/**
 * The option byte AREA itself, at 0x1FFF F800 (table 32-4): four words,
 * each holding two bytes and their inverses. Read-only, like everything
 * else this file does with the option bytes - it exists because two of
 * those bytes, Data0 and Data1, are the user's own two bytes of
 * provisioning and FLASH_OBR does not carry them on this family.
 *
 * `consistent()` is the check the loader makes: every byte against its
 * own inverse. It is the software answer to FLASH_OBR.OBERR and it can
 * be asked about the AREA when the register's answer is about the last
 * reset's load.
 */
struct FlashOptionArea {
    uint8_t rdpr;       ///< 0xA5 = the array is readable
    uint8_t user;       ///< IWDGSW, STOPRST, STANDYRST and the memory split in [7:5]
    uint8_t data0;      ///< two bytes of the user's own
    uint8_t data1;
    uint8_t wrpr[4];    ///< the write protection as it is stored

    /// Where the area answers (32.1's table).
    static constexpr uint32_t base = 0x1FFFF800UL;

    /// The four words as the core sees them, CONST: this driver reads
    /// the option bytes and never writes one, so the only view it hands
    /// out is one a store cannot go through. What writing them would
    /// cost is in the file header.
    static const volatile uint32_t* raw() {
        return reinterpret_cast<const volatile uint32_t*>(base);
    }

    static FlashOptionArea read() {
        const volatile uint32_t* w = reinterpret_cast<const volatile uint32_t*>(base);
        const uint32_t w0 = w[0];
        const uint32_t w1 = w[1];
        const uint32_t w2 = w[2];
        const uint32_t w3 = w[3];
        return FlashOptionArea{
            static_cast<uint8_t>(w0),
            static_cast<uint8_t>(w0 >> 16),
            static_cast<uint8_t>(w1),
            static_cast<uint8_t>(w1 >> 16),
            {static_cast<uint8_t>(w2), static_cast<uint8_t>(w2 >> 16),
             static_cast<uint8_t>(w3), static_cast<uint8_t>(w3 >> 16)},
        };
    }

    /// Every byte beside its inverse, as the loader checks them.
    static bool consistent() {
        const volatile uint32_t* w = reinterpret_cast<const volatile uint32_t*>(base);
        for (uint32_t i = 0; i < 4u; ++i) {
            const uint32_t v = w[i];
            const uint8_t lo = static_cast<uint8_t>(v);
            const uint8_t lo_n = static_cast<uint8_t>(v >> 8);
            const uint8_t hi = static_cast<uint8_t>(v >> 16);
            const uint8_t hi_n = static_cast<uint8_t>(v >> 24);
            if (static_cast<uint8_t>(lo ^ lo_n) != 0xFFu ||
                static_cast<uint8_t>(hi ^ hi_n) != 0xFFu) {
                return false;
            }
        }
        return true;
    }

    /// USER[7:5] as it stands in flash - what the NEXT reset will load.
    constexpr uint8_t split_code() const {
        return static_cast<uint8_t>(user >> flash_user_split_shift);
    }

    /// The same decode `FlashOptions::split()` makes, of the stored byte.
    constexpr std::optional<FlashSplit> split() const {
        return flash_split_of(device::flash_split_table, split_code());
    }
};

// =============================================================================
// The electronic signature (RM ch. 31)
// =============================================================================

/**
 * The 96-bit identifier the factory programmed into the system memory,
 * as three words in the order the chapter numbers them. Read-only on
 * any width (31.1); `device.hpp`'s `esig_uid_word()` is the accessor.
 */
struct DeviceUid {
    uint32_t word[3];

    static DeviceUid read() {
        return DeviceUid{{esig_uid_word(0), esig_uid_word(1), esig_uid_word(2)}};
    }
};

/**
 * ESIG_FLACAP: "the capacity of the flash memory area" in kilobytes, as
 * the factory wrote it (31.2.1). What it counts is not the window the
 * option byte selected but the LARGEST one it can select - 288 on a
 * CH32V303VC whose split is 256 - so the number it is compared with is
 * `flash_window_max_bytes`, which on a part with no split is the window
 * itself (the file header).
 */
inline uint16_t flash_size_kbytes() { return esig_flash_kbytes(); }

// =============================================================================
// The engine
// =============================================================================

/// What `erase_chip()` wants to be handed before it does what its name
/// says. There is no address in that operation and no part of the array
/// it spares, so the argument is the whole of the safety: a default of
/// `refuse` means a call that says nothing does nothing.
enum class ChipErasePolicy : uint8_t {
    refuse = 0,                    ///< the default: the verb returns `refused` and writes nothing
    erase_the_running_image = 1,   ///< spelled out, because that is what MER does
};

/**
 * The flash program and erase engine, monostate.
 *
 *   using Clk = brio::Clock<brio::ClockSource::pll, 96'000'000>;
 *   brio::Flash::unlock();
 *   const uint32_t err = brio::Flash::erase_page(Clk{}, 0xF000);
 *   brio::Flash::lock();
 *
 * EVERY WRITE VERB RETURNS A MASK, not a bool: zero is success, the low
 * bits are FLASH_STATR's own (`flash_wrprterr`, `flash_bsy` for an
 * operation that never finished) and the two top bits are this
 * driver's - `refused` for a contract the CALLER broke (an address that
 * is not aligned, a span that is not a page, a run past the array or
 * across the line between the window and the tail, a policy not
 * passed) and `refused_rate` for a rate the SILICON will not write at.
 * They are distinct on purpose: a refusal is never mistaken for a flash
 * error.
 *
 * EVERY ADDRESS-TAKING VERB HAS A SECOND SPELLING with the address as a
 * template argument - `erase_sector<a>(clock)` beside `erase_sector(clock,
 * a)` - where the refusal is a COMPILE error: a tail address on a part
 * that has none, a run past the array.
 *
 * THE UNLOCK WINDOW IS THE CALLER'S. unlock() and lock() are separate
 * verbs because a medium writing several pages wants one window for all
 * of them, and because a program that keeps the engine unlocked between
 * operations is making a decision worth seeing. No verb here unlocks
 * behind its caller's back.
 *
 * A WRITE IS A WAIT AND NOT A STALL. Every erase and program polls BSY,
 * so it blocks its CALLER for the milliseconds the datasheet's table
 * 4-17 gives it - and nothing else: measured on the CH32V203C8 and the
 * CH32V303VC, the core goes on executing out of the window while the
 * engine erases a page of it or a sector of the tail, the kernel's tick
 * advancing by what a timer measured across the erase, so interrupts
 * keep being served throughout.
 */
struct Flash {
    Flash() = delete;

    // ---- the geometry ------------------------------------------------------
    /// The fast grain: what FTPG programs and FTER erases (32.2.1).
    static constexpr uint32_t page_size = device::flash_page_bytes;      // 256
    /// The standard erase grain, which is also the WRITE PROTECTION unit
    /// (32.1's note 1): sixteen pages.
    static constexpr uint32_t sector_size = device::flash_protect_bytes; // 4096
    /// What BER32 erases.
    static constexpr uint32_t block_size = 32UL * 1024UL;
    /// The standard programming grain (32.5.3): two bytes, at an even
    /// address, and "when any non-halfword data is written, FPEC will
    /// generate a bus error".
    static constexpr uint32_t half_word_size = 2;

    /// THE ARRAY'S THREE NUMBERS (the file header): the zero-wait window
    /// every linker script and the medium reckon with, the non-zero-wait
    /// tail above it, and the die's whole code flash.
    static constexpr uint32_t window_bytes = device::flash_bytes;
    static constexpr uint32_t tail_bytes = device::flash_tail_bytes;
    static constexpr uint32_t array_bytes = device::flash_array_bytes;
    static constexpr bool has_tail = tail_bytes != 0u;
    /// The tail's first address and one past its last, counted from zero
    /// like every address here: it starts where the window ends.
    static constexpr uint32_t tail_base = window_bytes;
    static constexpr uint32_t tail_end = window_bytes + tail_bytes;

    static_assert(tail_end <= array_bytes,
                  "brio Flash: the part table's window and tail do not fit its array");
    static_assert(window_bytes % sector_size == 0u && tail_bytes % sector_size == 0u,
                  "brio Flash: the window and the tail are whole sectors on every part");

    static constexpr uint32_t alias_base = flash_alias_base;   // 0x0000 0000, where the image runs
    static constexpr uint32_t array_base = flash_array_base;   // 0x0800 0000, where the engine writes

    /// What a read of an erased cell answers on this silicon (32.5.4,
    /// 32.5.7): NOT 0xFFFFFFFF. The byte reads the chapter lists are
    /// this word's own bytes, little end first.
    static constexpr uint32_t erased_word = 0xE339E339UL;

    /// How long a verb polls BSY before it gives up. An erase is
    /// sixteen milliseconds typical and a stuck BSY is a fact the caller
    /// must see rather than a hang.
    static constexpr uint32_t timeout_polls = 4'000'000UL;

    /// How many reads of FLASH_STATR.EHMODS the enhanced read mode's
    /// verb spends waiting for the status to follow the bit it wrote
    /// (32.3: "set the EHMOD bit ... and then read the EHMODS bit").
    /// A BOUND and not an expectation: on a part where the mode does
    /// not engage at all, this is what makes the verb answer instead of
    /// hanging.
    static constexpr uint32_t mode_spins = 64;

    /// The caller broke the contract: a misaligned address, a span that
    /// is not a page, a run past the array, a chip erase with no policy.
    /// A bit no STATR flag uses.
    static constexpr uint32_t refused = 1UL << 31;
    /// The RATE is too high for an erase, a program or a read of the
    /// tail (32.1): HCLK above `flash_safe_sysclk_hz` with a clock chosen
    /// at run time. Under a static Clock the same condition is a compile
    /// error instead.
    static constexpr uint32_t refused_rate = 1UL << 30;

    /// The engine's registers, for a program that has to stage a
    /// sequence this driver does not offer - the same accessor every
    /// resource of this stratum carries.
    static FlashRegs& regs() { return *flash_ctl(); }

    /// The address the ENGINE takes for the byte at `addr`, counted from
    /// the start of the array: the array's own address, 0x0800 0000
    /// above it - which is where the image sees a byte of the window at
    /// `addr` itself, and where the tail is read. Applied here and
    /// nowhere else.
    static constexpr uint32_t alias_of(uint32_t addr) { return array_base + addr; }

    /// Is the run [addr, addr + bytes) inside the zero-wait window?
    static constexpr bool in_window(uint32_t addr, uint32_t bytes) {
        return addr <= window_bytes && bytes <= window_bytes - addr;
    }

    /// Is it inside the non-zero-wait tail? Never on a part that has
    /// none.
    static constexpr bool in_tail(uint32_t addr, uint32_t bytes) {
        return has_tail && addr >= tail_base && addr <= tail_end && bytes <= tail_end - addr;
    }

    /// Is it wholly in one of the two - where every write verb works?
    /// A run across the line between them is in neither.
    static constexpr bool in_array(uint32_t addr, uint32_t bytes) {
        return in_window(addr, bytes) || in_tail(addr, bytes);
    }

    // ---- the locks (32.5.2, 32.5.5, 32.6.1) --------------------------------
    static bool locked() { return (flash_ctl()->CTLR & flash_lock) != 0u; }
    static bool fast_locked() { return (flash_ctl()->CTLR & flash_flock) != 0u; }
    /// OBWRE is the option lock read the other way round: SET means the
    /// option bytes may be written.
    static bool options_locked() { return (flash_ctl()->CTLR & flash_obwre) == 0u; }

    /**
     * Open both locks the main array needs - the FPEC's and the fast
     * mode's - each with its own key pair, in order and consecutively.
     * False when either stays shut, which after a wrong sequence is
     * until the next system reset (32.5.2): not something to retry.
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

    /// The FPEC lock alone, for a program that only ever uses the
    /// standard half-word and 4 KB verbs.
    static bool unlock_standard() {
        if (locked()) {
            flash_ctl()->KEYR = flash_key1;
            flash_ctl()->KEYR = flash_key2;
        }
        return !locked();
    }

    /// Close both: writing a one is what SETS each of them (RW1), so
    /// this is a plain store of the two bits over the rest of CTLR.
    static void lock() {
        flash_ctl()->CTLR = (flash_ctl()->CTLR & ~flash_ctlr_write_once) | flash_flock | flash_lock;
    }

    // ---- the status (32.4.3) -----------------------------------------------
    static uint32_t status() { return flash_ctl()->STATR; }
    static bool busy() { return (flash_ctl()->STATR & flash_bsy) != 0u; }
    /// WRBSY: the fast-program buffer is taking the word just stored.
    /// "If this bit is '0', it means that the next data is allowed to be
    /// written."
    static bool write_busy() { return (flash_ctl()->STATR & flash_wrbsy) != 0u; }
    static bool operation_ended() { return (flash_ctl()->STATR & flash_eop) != 0u; }
    static uint32_t errors() { return flash_ctl()->STATR & flash_errors; }

    /// EOP and WRPRTERR, both write-one-to-clear: a plain store, never a
    /// read-modify-write, because a zero is the leave-it-alone value.
    static void clear_flags() { flash_ctl()->STATR = flash_statr_w1c; }

    // ---- the interrupt (32.4.4's EOPIE and ERRIE, Irq::flash) ---------------
    /// Arm the end-of-operation interrupt, the error one, neither or
    /// both. The vector is the caller's to enable (`Pfic::enable`).
    static void interrupts(bool on_end, bool on_error) {
        uint32_t ctlr = flash_ctl()->CTLR & ~(flash_ctlr_write_once | flash_eopie | flash_errie);
        if (on_end) {
            ctlr |= flash_eopie;
        }
        if (on_error) {
            ctlr |= flash_errie;
        }
        flash_ctl()->CTLR = ctlr;
    }
    static bool end_interrupt() { return (flash_ctl()->CTLR & flash_eopie) != 0u; }
    static bool error_interrupt() { return (flash_ctl()->CTLR & flash_errie) != 0u; }

    /// The flash vector's body: the flags that stood, cleared, handed
    /// back for the program to read.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t flags = flash_ctl()->STATR & flash_statr_w1c;
        flash_ctl()->STATR = flags;
        return flags;
    }

    // ---- the access clock (32.1, 32.4.4's SCKMOD) --------------------------
    /// What the array is read at with SCKMOD as it stands: HCLK, or half
    /// of it.
    template <typename Clock>
    static uint32_t access_hz(Clock clock) {
        const uint32_t hclk = hclk_of(clock);
        return access_clock_whole() ? hclk : hclk / 2u;
    }

    static bool access_clock_whole() { return (flash_ctl()->CTLR & flash_sckmod) != 0u; }

    /**
     * SCKMOD: read the array at HCLK (`whole`) instead of at half of it.
     * The engine must be unlocked - 32.4.4 calls the bit "a normal
     * unlock + quick unlock" - and the rate must leave the access clock
     * at or under `flash_access_max_hz`, which is what makes this a
     * REFUSAL and not a plain setter. False, with nothing written, when
     * either condition fails.
     */
    template <typename Clock>
    static bool access_clock_whole(Clock clock, bool whole) {
        if (whole && hclk_of(clock) > flash_access_max_hz) {
            return false;
        }
        if (locked() || fast_locked()) {
            return false;
        }
        uint32_t ctlr = flash_ctl()->CTLR & ~(flash_ctlr_write_once | flash_sckmod);
        if (whole) {
            ctlr |= flash_sckmod;
        }
        flash_ctl()->CTLR = ctlr;
        return access_clock_whole() == whole;
    }

    // ---- the enhanced read mode (32.3) -------------------------------------
    /// Whether the mode is in force, as the hardware reports it
    /// (FLASH_STATR.EHMODS). This is what every write verb refuses on.
    static bool enhanced_read() { return (flash_ctl()->STATR & flash_ehmods) != 0u; }

    /**
     * Enter or leave 32.3's enhanced read mode. Entering is EHMOD set
     * and then EHMODS read back; leaving is EHMOD cleared and RSENACT
     * written one, in that order and not the other. Needs both locks
     * open. Returns what EHMODS then says, so a caller never has to
     * guess.
     *
     * A program that enters it must leave it before any erase or
     * program (32.3's note 1) and before a Stop (note 2); a power or
     * system reset leaves it for free (note 3).
     */
    static bool enhanced_read(bool on) {
        if (locked() || fast_locked()) {
            return enhanced_read();
        }
        if (on) {
            flash_ctl()->CTLR = (flash_ctl()->CTLR & ~flash_ctlr_write_once) | flash_ehmod;
        } else {
            flash_ctl()->CTLR = flash_ctl()->CTLR & ~(flash_ctlr_write_once | flash_ehmod);
            flash_ctl()->CTLR = (flash_ctl()->CTLR & ~flash_ctlr_write_once) | flash_rsenact;
        }
        for (uint32_t i = 0; i < mode_spins && enhanced_read() != on; ++i) {
        }
        return enhanced_read();
    }

    // ---- reading ------------------------------------------------------------
    /// The WINDOW, through the alias the image runs at. The array is
    /// memory-mapped and answers on any width (32.5.1): this is a copy
    /// and nothing more, at any rate. The tail is `read_tail()`'s.
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        memcpy(dst.data(), reinterpret_cast<const void*>(alias_base + addr), dst.size());
    }

    /// Is the run at `addr` erased - that is, every word of it
    /// `erased_word`? The question a medium asks before it programs,
    /// and the reason `erased_word` is published. The window's, like
    /// `read()`.
    static bool erased(uint32_t addr, uint32_t bytes) {
        if ((addr % 4u) != 0u || (bytes % 4u) != 0u) {
            return false;
        }
        const volatile uint32_t* w = reinterpret_cast<const volatile uint32_t*>(alias_base + addr);
        for (uint32_t i = 0; i < bytes / 4u; ++i) {
            if (w[i] != erased_word) {
                return false;
            }
        }
        return true;
    }

    /**
     * A run of the TAIL, through the array's own address. `refused` for
     * a run that is not wholly in the tail (and on a part that has
     * none), `refused_rate` above `flash_safe_sysclk_hz` under a clock
     * chosen at run time - a compile error under a static one - because
     * the tail is read out of the array at the access clock (the file
     * header). Zero, and the bytes copied, otherwise.
     */
    template <typename Clock>
    static uint32_t read_tail(Clock clock, uint32_t addr, std::span<uint8_t> dst) {
        if (!in_tail(addr, static_cast<uint32_t>(dst.size()))) {
            return refused;
        }
        const uint32_t bad = tail_rate_refusal(clock);
        if (bad != 0u) {
            return bad;
        }
        memcpy(dst.data(), reinterpret_cast<const void*>(array_base + addr), dst.size());
        return 0u;
    }

    /// The same with the address a constant: a run outside the tail - on
    /// a part with no tail, any run - does not compile.
    template <uint32_t addr, typename Clock>
    static uint32_t read_tail(Clock clock, std::span<uint8_t> dst) {
        static_assert(has_tail,
                      "brio Flash: THIS PART HAS NO TAIL - its datasheet gives the non-zero-wait "
                      "area to other products (parts/<part>.hpp's flash_tail_bytes)");
        static_assert(in_tail(addr, 1u), "brio Flash: this address is not in the tail");
        return read_tail(clock, addr, dst);
    }

    // ---- erasing ------------------------------------------------------------
    /**
     * One 256-byte page, the fast way (32.5.7): FTER, the page's first
     * address into FLASH_ADDR, STRT. Both locks open, the page aligned,
     * in the window or the tail (the file header).
     */
    template <typename Clock>
    static uint32_t erase_page(Clock clock, uint32_t addr) {
        if ((addr % page_size) != 0u || !in_array(addr, page_size)) {
            return refused;
        }
        return run(clock, flash_fter, alias_of(addr), true, true);
    }

    template <uint32_t addr, typename Clock>
    static uint32_t erase_page(Clock clock) {
        static_assert((addr % page_size) == 0u, "brio Flash: a page address is a multiple of 256");
        reach<addr, page_size>();
        return erase_page(clock, addr);
    }

    /// One 4 KB sector, the standard way (32.5.4): PER, the sector's
    /// first address, STRT. Sixteen pages at a time, which is also one
    /// write-protection unit. A STANDARD verb, so the FPEC lock alone
    /// has to be open (32.4.4's own note on the bit). The window or the
    /// tail.
    template <typename Clock>
    static uint32_t erase_sector(Clock clock, uint32_t addr) {
        if ((addr % sector_size) != 0u || !in_array(addr, sector_size)) {
            return refused;
        }
        return run(clock, flash_per, alias_of(addr), true, false);
    }

    template <uint32_t addr, typename Clock>
    static uint32_t erase_sector(Clock clock) {
        static_assert((addr % sector_size) == 0u,
                      "brio Flash: a sector address is a multiple of 4096");
        reach<addr, sector_size>();
        return erase_sector(clock, addr);
    }

    /**
     * One 32 KB block, the fast way (32.5.7's second half): BER32. The
     * window or the tail, whose 224 KB are seven whole blocks.
     *
     * NOTHING PROTECTS THE RUNNING IMAGE HERE but the address the caller
     * passes. On the 32 KB parts of this series there is exactly one
     * such block and it holds the whole program; on the others, block
     * zero does.
     */
    template <typename Clock>
    static uint32_t erase_block32(Clock clock, uint32_t addr) {
        if ((addr % block_size) != 0u || !in_array(addr, block_size)) {
            return refused;
        }
        return run(clock, flash_ber32, alias_of(addr), true, true);
    }

    template <uint32_t addr, typename Clock>
    static uint32_t erase_block32(Clock clock) {
        static_assert((addr % block_size) == 0u,
                      "brio Flash: a block address is a multiple of 32768");
        reach<addr, block_size>();
        return erase_block32(clock, addr);
    }

    /**
     * The whole user array (32.5.4's figure 32-3): MER, STRT, no
     * address. It takes the running image with it, which is why it is
     * refused unless the caller SAYS SO - and why a bench suite never
     * passes the argument that lets it through.
     */
    template <typename Clock>
    static uint32_t erase_chip(Clock clock, ChipErasePolicy policy = ChipErasePolicy::refuse) {
        if (policy != ChipErasePolicy::erase_the_running_image) {
            return refused;
        }
        return run(clock, flash_mer, 0u, false, false);
    }

    // ---- programming --------------------------------------------------------
    /**
     * TWO BYTES, the standard way (32.5.3): PG set, the half-word stored
     * at an even address, BSY watched. The store IS the operation -
     * there is no start bit in this one - and a store of any other width
     * is a bus error, which is why the argument is a `uint16_t` and the
     * address is checked. The window or the TAIL.
     */
    template <typename Clock>
    static uint32_t program_half_word(Clock clock, uint32_t addr, uint16_t value) {
        if ((addr % half_word_size) != 0u || !in_array(addr, half_word_size)) {
            return refused;
        }
        const uint32_t bad = before_write(clock, false);
        if (bad != 0u) {
            return bad;
        }
        clear_flags();
        set_mode(flash_pg);
        *reinterpret_cast<volatile uint16_t*>(alias_of(addr)) = value;
        const uint32_t err = finish();
        clear_mode(flash_pg);
        return err;
    }

    template <uint32_t addr, typename Clock>
    static uint32_t program_half_word(Clock clock, uint16_t value) {
        static_assert((addr % half_word_size) == 0u,
                      "brio Flash: a half-word is programmed at an even address (RM 32.5.3)");
        reach<addr, half_word_size>();
        return program_half_word(clock, addr, value);
    }

    /**
     * ONE WHOLE PAGE, the fast way (32.5.6): FTPG set, then 64 words
     * stored into the array with WRBSY watched between them, then
     * PGSTRT. `src` is exactly `page_size` bytes and `addr` is page
     * aligned, in the window or the tail (the file header).
     *
     * FLASH_ADDR IS WRITTEN BEFORE PGSTRT although 32.5.6 does not list
     * that step - the sister family's chapter does list it for the same
     * operation, and the register is what the erase verbs use to say
     * WHERE. MEASURED, THE SILICON DOES NOT NEED IT, on the CH32V203C8
     * and the CH32V303VC: the same sequence with the store left out, and
     * the register pointing at another erased page, wrote the page the
     * sixty-four stores named and left the other one untouched. The
     * store stays because it costs one instruction and makes the
     * sequence the one both chapters describe.
     */
    template <typename Clock>
    static uint32_t program_page(Clock clock, uint32_t addr, std::span<const uint8_t> src) {
        if (src.size() != page_size || (addr % page_size) != 0u || !in_array(addr, page_size)) {
            return refused;
        }
        const uint32_t bad = before_write(clock, true);
        if (bad != 0u) {
            return bad;
        }
        clear_flags();
        set_mode(flash_ftpg);
        volatile uint32_t* cell = reinterpret_cast<volatile uint32_t*>(alias_of(addr));
        for (uint32_t i = 0; i < page_size / 4u; ++i) {
            uint32_t word;
            memcpy(&word, src.data() + i * 4u, 4u);
            cell[i] = word;
            if (!wait_write_idle()) {
                clear_mode(flash_ftpg);
                return flash_bsy;
            }
        }
        flash_ctl()->ADDR = alias_of(addr);
        flash_ctl()->CTLR = (flash_ctl()->CTLR & ~flash_ctlr_write_once) | flash_pgstrt;
        const uint32_t err = finish();
        clear_mode(flash_ftpg);
        return err;
    }

    template <uint32_t addr, typename Clock>
    static uint32_t program_page(Clock clock, std::span<const uint8_t> src) {
        static_assert((addr % page_size) == 0u, "brio Flash: a page address is a multiple of 256");
        reach<addr, page_size>();
        return program_page(clock, addr, src);
    }

    // ---- the option bytes and the signature, in one place -------------------
    static FlashOptions options() { return FlashOptions::read(); }
    /// FLASH_WPR as loaded: a CLEAR bit is a protected 4 KB sector.
    static uint32_t write_protection() { return flash_ctl()->WPR; }
    static bool read_protected() { return (flash_ctl()->OBR & flash_obr_rdprt) != 0u; }
    static DeviceUid uid() { return DeviceUid::read(); }
    static uint16_t size_kbytes() { return flash_size_kbytes(); }
    /// The memory split in force, decoded - nothing on a part with no
    /// split.
    static std::optional<FlashSplit> split() { return FlashOptions::read().split(); }

    /// HCLK as the clock names it, whichever kind of clock it is: a
    /// constant for a static one, this moment's rate for a dynamic one.
    template <typename Clock>
    static uint32_t hclk_of(Clock) {
        if constexpr (Clock::is_static) {
            return Clock::hz;
        } else {
            return Clock::hz();
        }
    }

private:
    /// The compile-time reach (the struct's comment): the window and the
    /// tail above it, for both methods alike.
    template <uint32_t addr, uint32_t bytes>
    static constexpr void reach() {
        static_assert(has_tail || addr < window_bytes,
                      "brio Flash: THIS PART HAS NO TAIL - its datasheet gives the non-zero-wait "
                      "area to other products, so the window is the whole of what a program may "
                      "write (parts/<part>.hpp's flash_tail_bytes)");
        static_assert((!has_tail && addr >= window_bytes) || in_array(addr, bytes),
                      "brio Flash: this run is past the array this part offers - the window and "
                      "the tail above it");
    }

    /// The rate half of the contract (the file header): a compile error
    /// under a static Clock that needs HCLK halved, a code under a
    /// dynamic one whose rate happens to be there now.
    template <typename Clock>
    static uint32_t rate_refusal(Clock clock) {
        if constexpr (Clock::is_static) {
            static_assert(!Clock::flash_needs_halving,
                          "brio Flash: an erase or a program at this rate wants HCLK HALVED "
                          "around it (RM 32.1 - the flash access clock may not exceed 60 MHz, "
                          "and SCKMOD's reset value already halves it), and this driver will "
                          "not move the clock tree behind its caller's back. Run the operation "
                          "at or below brio::flash_safe_sysclk_hz, which a DynamicClock can "
                          "step down to");
            (void)clock;
            return 0u;
        } else {
            return hclk_of(clock) > flash_safe_sysclk_hz ? refused_rate : 0u;
        }
    }

    /// The same rule for a READ of the tail, with its own words: the
    /// window reads at any rate and the tail does not (the file header).
    template <typename Clock>
    static uint32_t tail_rate_refusal(Clock clock) {
        if constexpr (Clock::is_static) {
            static_assert(!Clock::flash_needs_halving,
                          "brio Flash: a READ of the non-zero-wait tail at this rate wants HCLK "
                          "HALVED around it (RM 32.1's note 2 names the non-zero-wait area among "
                          "the flash operations that do, and the tail is read out of the array at "
                          "the access clock, 60 MHz at most); the window reads at any rate. Read "
                          "the tail at or below brio::flash_safe_sysclk_hz, which a DynamicClock "
                          "can step down to");
            (void)clock;
            return 0u;
        } else {
            return hclk_of(clock) > flash_safe_sysclk_hz ? refused_rate : 0u;
        }
    }

    /**
     * Everything that must hold before the engine is told to do
     * anything: the rate, the enhanced read mode, the locks, a previous
     * operation.
     *
     * WHICH LOCKS depends on the verb, and 32.4.4 says so bit by bit:
     * PG, PER, MER and STRT are "a normal unlock", FTPG, FTER and BER32
     * are "a normal unlock + quick unlock". So `fast` is what the
     * caller passes, and a program that opened only the FPEC lock
     * (`unlock_standard()`) can still write half-words and erase
     * sectors.
     */
    template <typename Clock>
    static uint32_t before_write(Clock clock, bool fast) {
        const uint32_t bad = rate_refusal(clock);
        if (bad != 0u) {
            return bad;
        }
        if (enhanced_read()) {
            return refused;
        }
        if (locked() || (fast && fast_locked())) {
            return refused;
        }
        return wait_idle() ? 0u : flash_bsy;
    }

    /// The shape every erase shares: the mode bit, an address where
    /// there is one, STRT, the wait, the mode bit down again.
    template <typename Clock>
    static uint32_t run(Clock clock, uint32_t mode, uint32_t alias_addr, bool has_address,
                        bool fast) {
        const uint32_t bad = before_write(clock, fast);
        if (bad != 0u) {
            return bad;
        }
        clear_flags();
        set_mode(mode);
        if (has_address) {
            flash_ctl()->ADDR = alias_addr;
        }
        flash_ctl()->CTLR = (flash_ctl()->CTLR & ~flash_ctlr_write_once) | flash_strt;
        const uint32_t err = finish();
        clear_mode(mode);
        return err;
    }

    static void set_mode(uint32_t bit) {
        flash_ctl()->CTLR = (flash_ctl()->CTLR & ~flash_ctlr_write_once) | bit;
    }
    static void clear_mode(uint32_t bit) {
        flash_ctl()->CTLR = flash_ctl()->CTLR & ~(flash_ctlr_write_once | bit);
    }

    static bool wait_idle() {
        for (uint32_t i = 0; i < timeout_polls; ++i) {
            if (!busy()) {
                return true;
            }
        }
        return false;
    }

    static bool wait_write_idle() {
        for (uint32_t i = 0; i < timeout_polls; ++i) {
            if (!write_busy()) {
                return true;
            }
        }
        return false;
    }

    /// Wait for the operation, then read what it left: the error bits,
    /// or zero. EOP is cleared on the way out.
    static uint32_t finish() {
        if (!wait_idle()) {
            return flash_bsy;
        }
        const uint32_t err = flash_ctl()->STATR & flash_errors;
        clear_flags();
        return err;
    }
};

} // namespace brio
