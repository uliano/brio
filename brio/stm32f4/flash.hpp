/*
 * flash.hpp
 *
 * The embedded flash memory interface (RM0090 ch. 3, RM0390 ch. 3,
 * RM0383 ch. 3): the read side the CLOCK TASK needs - the access
 * LATENCY and the ART accelerator's three switches - and the write
 * side, the program and erase engine with the option bytes above it.
 * One chapter, one file.
 *
 * WHY THE WAIT STATES LIVE HERE AND NOT IN clock.hpp: they are the
 * flash interface's register. The clock task calls in; this file owns
 * the register.
 *
 * WHAT THIS FILE IS NOT: a FlashMedia. There is no storage backend on
 * this family and no partition in any linker script, because the
 * geometry pushes back - the small sectors are at the BOTTOM of the
 * bank, under the vector table, and everything above 128 Kbytes is one
 * 128 Kbyte sector - and because the question that raises is prior to
 * the answer (CLAUDE.md, "The NV stack reviewed as a whole"). What is
 * here is the engine; who stores what in it is not decided.
 *
 * THE CHAPTER, in the shape the code takes:
 *
 *  - ONE ARRAY, UNEVEN SECTORS, ONE ERASE GRAIN. A bank's sectors are
 *    four of 16 Kbytes, then one of 64, then 128 Kbytes to the end of
 *    the bank - the same rule in all four chapters read, which is why
 *    the map below is COMPUTED from the flash size register and not
 *    tabulated. The sector is the only erase unit there is; there is no
 *    page and no row erase.
 *
 *  - THE PROGRAM UNIT IS THE ACCESS WIDTH, and it is declared twice: in
 *    FLASH_CR.PSIZE and in the STORE INSTRUCTION, which must agree or
 *    the silicon raises PGPERR (3.5.4). That is why the parallelism is a
 *    TEMPLATE parameter here - it decides which instruction the compiler
 *    emits - and not a runtime argument that would need a switch over
 *    four widths inside the loop.
 *
 *  - PSIZE IS A CLAIM ABOUT THE SUPPLY (table 6 / table 13): x8 down to
 *    1.7 V, x16 from 2.1, x32 from 2.7, and x64 ONLY with an external
 *    8..9 V supply on the VPP pad. `x64` is refused at compile time:
 *    no board of this bench has that pad wired, the datasheet warns
 *    that VPP must not stay applied for more than an hour, and "any
 *    program or erase operation started with inconsistent parallelism
 *    /voltage settings may lead to unpredicted results. Even if a
 *    subsequent read indicates that the logical value was effectively
 *    written, this value may not be retained." The other three widths
 *    are the caller's word about ITS board, which no register can check.
 *
 *  - A CELL IS NOT WRITTEN ONCE. 3.5.4: "Successive write operations are
 *    possible without the need of an erase operation when changing bits
 *    from 1 to 0. Writing 1 requires a flash memory erase operation."
 *    So a second program to a programmed word ANDs into it and raises no
 *    error - the opposite of the STM32G0's PROGERR and of what
 *    util/nv_heap.hpp's write-once cell contract assumes. Measured, and
 *    the number is in docs/stm32f4/flash.md.
 *
 *  - THE CORE IS FROZEN WHILE THE FLASH WORKS. 3.5: "Any attempt to read
 *    the flash memory while it is being written or erased causes the bus
 *    to stall" - and on a single-bank part the code IS in the flash, so
 *    the instruction fetches stop with it: no interrupt is taken, the
 *    kernel tick loses every millisecond of a 128 Kbyte erase. The
 *    F42x/F43x class is the exception: two banks, and a read of one runs
 *    while the other is erased (3.6.5). `last_wait_turns()` is what
 *    measures which of the two a part is doing - the count of poll
 *    iterations the CPU completed while the engine was busy.
 *
 *  - THE UNLOCK IS A KEYED SEQUENCE AND A WRONG ONE IS FATAL UNTIL THE
 *    NEXT RESET. 3.5.1: KEY1 then KEY2 into FLASH_KEYR; "any wrong
 *    sequence returns a bus error and lock up the FLASH_CR register
 *    until the next reset". So unlock() writes the pair only when LOCK
 *    really stands, and no verb in this file can produce half a
 *    sequence.
 *
 *  - EOP EXISTS ONLY IF ITS INTERRUPT IS ENABLED. 3.8.4 bit 0: EOP "is
 *    set only if the end of operation interrupts are enabled (EOPIE=1)",
 *    and bit 1 says the same of OPERR and ERRIE - which the erase and
 *    program procedures of 3.5.3 and 3.5.4 do not mention, telling the
 *    programmer to wait on BSY instead. This file judges an operation by
 *    BSY falling and by the error flags; EOP is reported, never required.
 *
 *  - THE OPTION BYTES ARE A SECOND ENGINE with its own key, its own lock
 *    and its own start bit, and writing them ERASES AND REPROGRAMS the
 *    whole user configuration sector (3.6.2's note). RDP IS READ HERE
 *    AND NEVER WRITTEN BY ANY VERB: level 2 is irreversible and takes
 *    the debug port with it, and a level 1 that came from a slip costs a
 *    mass erase to undo. The one verb that does write an option -
 *    FlashOptions::protect() - reaches the register through HALF-WORD
 *    stores to its upper half, so the RDP byte is not in the data path
 *    at all, and it refuses unless RDP reads 0xAA and PCROP is off,
 *    which is the state figure 4 makes reversible.
 *
 *  - THE OTP AREA (3.7) is 512 bytes in sixteen blocks with a lock byte
 *    each, writable once and NEVER erasable. It is memory-mapped and
 *    readable like any other flash; this file exposes the read and, on
 *    purpose, no write: one wrong store is permanent.
 *
 * ERRATA. Nothing in ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6 is
 * filed against the program/erase engine or the option bytes. Two items
 * touch the chapter from outside and both are DUAL-BANK ONLY, so
 * neither is live on a single-bank part: ES0206 2.2.15 and ES0298 2.2.12
 * (the data cache may be corrupted when a read of one bank meets a write
 * of the other - the workaround is to drop DCEN around the write and
 * reset the cache after, which is what FlashAccel::flush_caches() is
 * for) and ES0206 2.2.14 (on the F42x/F43x with the second bank in use,
 * PA12 as a GPIO corrupts data read from the flash; a pad rule, not a
 * driver one). RM0383 3.5.4 and its twins ask for the same flush after
 * an erase that took down cached code, erratum or no erratum.
 *
 * Facts that shape the wait-state code (RM0090 3.5.1 and table 12,
 * RM0383 3.4.1):
 *  - the wait states are a function of HCLK AND of the supply voltage;
 *    stm32f4/device_tables.hpp carries each part class's 2.7..3.6 V
 *    column (0 WS up to 30 MHz, one more per 30 MHz band, 5 WS at
 *    180 MHz on the F42x/F43x and F446; the F411's bands are 30, 64,
 *    90 and 100 MHz). Out of reset HCLK is 16 MHz at 0 WS;
 *  - the ORDER: to increase the frequency, program the new latency,
 *    "check that the new number of wait states is taken into account
 *    by reading the FLASH_ACR register", then change the clock; to
 *    decrease it, change the clock first and the latency after (3.5.1's
 *    two sequences). set() below does the readback;
 *  - the ART ACCELERATOR (3.5.2): an instruction prefetch buffer
 *    (PRFTEN), a 64-line instruction cache (ICEN) and an 8-line data
 *    cache (DCEN), all three CLEAR at reset. With them off a 5-wait-
 *    state core loses most of its speed to the flash; the clock task
 *    turns all three on when it raises the rate. A cache may be RESET
 *    (ICRST, DCRST) only while it is disabled (3.9.1);
 *  - the LATENCY field is four bits on the parts whose ladder reaches
 *    180 MHz and three on the others (FLASH_ACR_LATENCY_Msk says which;
 *    max_latency below reads it).
 *
 * The errata sheets of the three bench parts carry no item against the
 * accelerator or the latency; the prefetch is therefore ON by default
 * here where the STM32G0's stratum leaves it off (that family's erratum
 * has no twin on this one).
 */
#pragma once

#include <stdint.h>

#include <optional>
#include <span>

#include "stm32f4xx.h"

#include "stm32f4/device_tables.hpp"

namespace brio {

/// FLASH_ACR.LATENCY - the read wait states, and the rule that picks
/// them. Monostate: one flash interface per device.
struct FlashWaitStates {
    FlashWaitStates() = delete;

    /// The field's own ceiling: 15 where LATENCY is four bits, 7 where
    /// it is three.
    static constexpr uint8_t max_latency = static_cast<uint8_t>(FLASH_ACR_LATENCY_Msk >> FLASH_ACR_LATENCY_Pos);

    static uint8_t get() {
        return static_cast<uint8_t>((FLASH->ACR & FLASH_ACR_LATENCY_Msk) >> FLASH_ACR_LATENCY_Pos);
    }

    /// Program `ws` and wait until it reads back (3.5.1). Bounded: a
    /// value the field cannot take is refused instead, and false says so.
    static bool set(uint8_t ws) {
        if (ws > max_latency) {
            return false;
        }
        FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) |
                     (static_cast<uint32_t>(ws) << FLASH_ACR_LATENCY_Pos);
        for (uint32_t spins = 0; spins < 1000u; ++spins) {
            if (get() == ws) {
                return true;
            }
        }
        return false;
    }

    /// The wait states `hclk` needs on this part at 2.7..3.6 V, from
    /// the reserve's ladder; 0xFF when the ladder is not known or the
    /// rate is beyond it (the clock task refuses at compile time first).
    static constexpr uint8_t needed_for(uint32_t hclk) { return flash_wait_states_for(hclk); }
};

/// The ART accelerator's switches (3.5.2, 3.9.1).
struct FlashAccel {
    FlashAccel() = delete;

    static void prefetch(bool on) { bit(FLASH_ACR_PRFTEN, on); }
    static bool prefetch() { return (FLASH->ACR & FLASH_ACR_PRFTEN) != 0u; }

    static void icache(bool on) { bit(FLASH_ACR_ICEN, on); }
    static bool icache() { return (FLASH->ACR & FLASH_ACR_ICEN) != 0u; }

    static void dcache(bool on) { bit(FLASH_ACR_DCEN, on); }
    static bool dcache() { return (FLASH->ACR & FLASH_ACR_DCEN) != 0u; }

    /// Reset the instruction cache: legal only while it is disabled
    /// (3.9.1), so a request on an enabled cache is refused.
    static bool icache_reset() {
        if (icache()) {
            return false;
        }
        FLASH->ACR |= FLASH_ACR_ICRST;
        FLASH->ACR &= ~FLASH_ACR_ICRST;
        return true;
    }
    static bool dcache_reset() {
        if (dcache()) {
            return false;
        }
        FLASH->ACR |= FLASH_ACR_DCRST;
        FLASH->ACR &= ~FLASH_ACR_DCRST;
        return true;
    }

    /**
     * Flush both caches and leave them as they were found - the whole
     * dance, because 3.5.4 allows the reset bits only while the cache is
     * disabled and an erase that took down CACHED code needs them:
     * "you have to make sure that these data are rewritten before they
     * are accessed during code execution. If this cannot be done safely,
     * it is recommended to flush the caches".
     *
     * It is also the workaround ES0206 2.2.15 and ES0298 2.2.12 ask for
     * around a write on a dual-bank part, where the two of them are the
     * same sentence: drop DCEN, write, reset the cache, enable it again.
     * Nothing here does it behind the caller's back - an erase does not
     * know which lines it invalidated, and a driver that flushes the
     * instruction cache on its own decides the system's timing.
     */
    static void flush_caches() {
        const bool i_was_on = icache();
        const bool d_was_on = dcache();
        icache(false);
        dcache(false);
        FLASH->ACR |= FLASH_ACR_ICRST | FLASH_ACR_DCRST;
        FLASH->ACR &= ~(FLASH_ACR_ICRST | FLASH_ACR_DCRST);
        icache(i_was_on);
        dcache(d_was_on);
    }

    /// All three on: what the clock task does when it raises the rate.
    static void enable_all() {
        FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    }

private:
    static void bit(uint32_t mask, bool on) {
        if (on) {
            FLASH->ACR |= mask;
        } else {
            FLASH->ACR &= ~mask;
        }
    }
};

/// The device electronic signature (RM0090 ch. 39, RM0390 ch. 34,
/// RM0383 ch. 24): the 96-bit unique ID, the flash size in Kbytes and
/// the package code, in system memory at the addresses the header
/// states. Read-only by nature.
struct DeviceUid {
    uint32_t word[3];

    static DeviceUid read() {
        const volatile uint32_t* p = reinterpret_cast<const volatile uint32_t*>(device_uid_base);
        return DeviceUid{{p[0], p[1], p[2]}};
    }
};

/// FLASHSIZE: the device's flash in Kbytes (0x200 = 512 K, 0x800 = 2 M).
inline uint16_t flash_size_kbytes() {
    return *reinterpret_cast<const volatile uint16_t*>(flash_size_register);
}

/// DBGMCU_IDCODE (RM0090 38.6.1): DEV_ID names the part class (0x419 the
/// F42x/F43x, 0x421 the F446, 0x431 the F411), REV_ID the silicon
/// revision the errata sheets key on.
struct DeviceIdcode {
    uint16_t dev_id;
    uint16_t rev_id;

    static DeviceIdcode read() {
        const uint32_t v = DBGMCU->IDCODE;
        return DeviceIdcode{static_cast<uint16_t>(v & DBGMCU_IDCODE_DEV_ID_Msk),
                            static_cast<uint16_t>((v & DBGMCU_IDCODE_REV_ID_Msk) >> DBGMCU_IDCODE_REV_ID_Pos)};
    }
};

// ---- the write side ---------------------------------------------------------

/**
 * FLASH_CR.PSIZE - how many bytes one high-voltage pulse programs, and
 * therefore how wide every store into the array must be (3.5.2, 3.5.4).
 *
 * It is a claim about the SUPPLY and not a speed setting: table 6 gives
 * x8 from 1.7 V, x16 from 2.1, x32 from 2.7, and x64 only with the
 * external VPP supply. It also decides the ERASE time - a 128 Kbyte
 * sector is 2 s at x8 and 1 s at x32 on the STM32F411 (DS10314 table 45).
 */
enum class FlashParallelism : uint8_t {
    x8 = 0,   ///< byte stores, 1.7 V and up
    x16 = 1,  ///< half-word stores, 2.1 V and up
    x32 = 2,  ///< word stores, 2.7 V and up
    x64 = 3,  ///< double-word stores, and ONLY with 8..9 V on VPP
};

/// How many bytes one pulse of that parallelism writes.
constexpr uint8_t flash_parallelism_bytes(FlashParallelism p) {
    return static_cast<uint8_t>(1u << static_cast<uint8_t>(p));
}

/// The lowest VDD the datasheet allows that parallelism at, in
/// millivolts; 0 for x64, whose condition is not a VDD at all but the
/// external supply on VPP. Nothing here reads it - the driver cannot
/// measure VDD - but a board file can state its rail and check.
constexpr uint16_t flash_parallelism_min_mv(FlashParallelism p) {
    switch (p) {
        case FlashParallelism::x8: return 1700;
        case FlashParallelism::x16: return 2100;
        case FlashParallelism::x32: return 2700;
        default: return 0;
    }
}

/// Which bank an operation names. One bank is the only bank on every
/// part but the F42x/F43x class.
enum class FlashBank : uint8_t { bank1 = 0, bank2 = 1 };

/**
 * The confirmation `Flash::mass_erase()` takes as a template argument.
 *
 * A mass erase takes down every user sector, the running image with it,
 * and there is no way for the engine to be politely wrong about that.
 * Making the confirmation a compile-time value means a suite cannot
 * reach it by a typo in a runtime expression: the call does not compile
 * unless the source says `erase_every_user_sector` in as many letters
 * (test/family_stm32f4/neg/flash_mass_erase_unconfirmed.cpp).
 */
enum class FlashEraseAll : uint8_t {
    no = 0,
    erase_every_user_sector = 1,
};

/**
 * FLASH_SR, bit by bit, as one mask vocabulary.
 *
 * Every operation in this file returns a MASK of these rather than an
 * enumerated single cause, because more than one can stand at once: a
 * program loop that meets PGPERR on its third word may already carry
 * WRPERR from its first, and 3.5.5's note warns that "if several
 * successive errors are detected the error flags cannot be cleared
 * until the end of the successive write requests".
 *
 * TWO SPELLINGS DIFFER FROM THE MANUAL'S, and the header is what
 * compiles: bit 1 is OPERR in every reference manual and
 * `FLASH_SR_SOP` in every device header of the pack, and bit 15 of
 * FLASH_CR is MER1 in RM0090 3.9.8 and `FLASH_CR_MER2` in the header.
 * Same bits, and the names below are the manual's.
 *
 * `refused` is NOT a silicon bit. Bits 31:17 of FLASH_SR are Reserved,
 * so this file borrows the top one to mean "this driver refused before
 * the flash was touched" - a misaligned address, a locked FLASH_CR, a
 * sector this part has not got, a wait that timed out. It can never
 * collide with a real flag, and a caller that only wants the hardware's
 * opinion masks it off with `errors`.
 */
struct FlashFlag {
    FlashFlag() = delete;

    /// End of operation - and only with EOPIE set (3.8.4 bit 0).
    static constexpr uint32_t eop = FLASH_SR_EOP;
    /// OPERR, the manual's name for the bit the header spells SOP - and
    /// only with ERRIE set (3.8.4 bit 1).
    static constexpr uint32_t operation_error = FLASH_SR_SOP;
    static constexpr uint32_t write_protect_error = FLASH_SR_WRPERR;
    /// The data would cross a 128-bit row (3.5.4).
    static constexpr uint32_t alignment_error = FLASH_SR_PGAERR;
    /// The store's width is not the one PSIZE declares (3.5.4).
    static constexpr uint32_t parallelism_error = FLASH_SR_PGPERR;
    /// A store into the array with PG clear, or a start with neither
    /// SER nor MER (3.5.4).
    static constexpr uint32_t sequence_error = FLASH_SR_PGSERR;
    /// RDERR, a D-bus read of a PCROPed sector - 0 where the part has
    /// no PCROP at all (the F405/F407/F415/F417 class).
    static constexpr uint32_t read_protect_error = flash_read_error_flag();

    /// Read-only: an operation is in progress.
    static constexpr uint32_t busy = FLASH_SR_BSY;

    /// This file's own: nothing was written to the flash at all.
    static constexpr uint32_t refused = 0x8000'0000UL;

    /// Everything that says an operation went wrong.
    static constexpr uint32_t errors = operation_error | write_protect_error | alignment_error |
                                       parallelism_error | sequence_error | read_protect_error;

    /// Everything a store of ones takes down.
    static constexpr uint32_t clearable = errors | eop;
};

/// One sector of the array: the manual's own sector NUMBER (which is
/// what FLASH_CR.SNB encodes, through flash_sector_number_code), the
/// bank it belongs to, and where it is.
struct FlashSector {
    uint8_t number = 0;
    uint8_t bank = 1;
    uint32_t base = 0;
    uint32_t size = 0;

    /// Whether `addr` falls inside this sector.
    constexpr bool contains(uint32_t addr) const {
        return addr >= base && addr - base < size;
    }

    constexpr bool operator==(const FlashSector&) const = default;
};

/**
 * The program and erase engine (RM0090 3.5 and 3.6, RM0390 3.5,
 * RM0383 3.5).
 *
 * Monostate: there is one flash interface and it is a piece of
 * hardware. THE RETURN VALUE OF EVERY OPERATION IS A MASK, zero meaning
 * success (FlashFlag). The flags an operation raised are also cleared
 * out of FLASH_SR on the way out, so the next operation cannot inherit
 * them - and the whole register as it stood is kept in last_status()
 * for a caller that wants EOP and BSY too.
 *
 * WHAT IS NOT HERE, each with its reason:
 *  - no OTP write. One store is permanent; there is no user and no way
 *    back. read_otp() is the half that is safe.
 *  - no RDP verb of any kind. Level 2 is irreversible and level 1 costs
 *    a mass erase to leave; provisioning belongs to a tool over the
 *    debug port, not to firmware.
 *  - no FlashMedia backend (see the file header).
 */
struct Flash {
    Flash() = delete;

    // ---- what this part is ------------------------------------------------

    /// The main array's address. The same memory is aliased at 0 when
    /// the part boots from flash; every verb here speaks the 0x0800_0000
    /// address, which is the one FLASH_CR's engine and the linker agree
    /// on.
    static constexpr uint32_t base = FLASH_BASE;

    static constexpr uint32_t key1 = 0x4567'0123UL;
    static constexpr uint32_t key2 = 0xCDEF'89ABUL;
    static constexpr uint32_t option_key1 = 0x0819'2A3BUL;
    static constexpr uint32_t option_key2 = 0x4C5D'6E7FUL;

    /// Whether this part class's chapter 3 was read - and so whether
    /// there is a sector map at all. Programming does not need one.
    static constexpr bool geometry_known = flash_facts().known;
    static constexpr bool dual_bank_capable = flash_facts().dual_bank_capable;

    /**
     * The bound on every wait, in poll iterations.
     *
     * It is NOT a time bound and cannot be: while the flash works, a
     * single-bank part's core is frozen and the loop turns not at all,
     * while a part running out of the other bank (or out of the
     * instruction cache) turns it at full speed. The worst case is the
     * second one over the longest operation the datasheet lists - an
     * 11 s mass erase at x8 (DS10314 table 45) - so a thousand million
     * turns is many times over it and still finite, which is the whole
     * point: a sick chip must give the caller a false, not a mute board.
     */
    static constexpr uint32_t wait_spins = 1'000'000'000UL;

    static uint32_t size_bytes() { return static_cast<uint32_t>(flash_size_kbytes()) * 1024u; }

    static bool in_main_flash(uint32_t addr) {
        return addr >= base && addr - base < size_bytes();
    }

    /**
     * How many banks the array is in RIGHT NOW. Two only on the
     * F42x/F43x class, and there only when the size says so: a 2 Mbyte
     * part is always dual bank, a 512 Kbyte one never is, and the
     * 1 Mbyte one is what OPTCR.DB1M decides (RM0090 3.4 and table 7).
     */
    static uint8_t bank_count() {
        if (!dual_bank_capable) {
            return 1;
        }
        const uint32_t kb = flash_size_kbytes();
        if (kb > 1024u) {
            return 2;
        }
        if (kb == 1024u) {
            return (FLASH->OPTCR & flash_dual_bank_option_mask()) != 0u ? 2 : 1;
        }
        return 1;
    }

    static uint32_t bank_bytes() { return size_bytes() / bank_count(); }

    /// The size of the i-th sector of a bank, counted from its bottom:
    /// four of 16 Kbytes, one of 64, then 128 Kbytes - the rule every
    /// chapter read states.
    static constexpr uint32_t sector_size_in_bank(uint8_t i) {
        if (i < flash_small_sectors) {
            return flash_small_sector_bytes;
        }
        if (i == flash_small_sectors) {
            return flash_middle_sector_bytes;
        }
        return flash_large_sector_bytes;
    }

    /// How many sectors fit in one bank of this part. Zero where the
    /// map is not known.
    static uint8_t sectors_per_bank() {
        if (!geometry_known) {
            return 0;
        }
        const uint32_t bank = bank_bytes();
        uint32_t at = 0;
        uint8_t i = 0;
        while (at < bank && i < flash_facts().max_sectors_per_bank) {
            at += sector_size_in_bank(i);
            ++i;
        }
        return at == bank ? i : 0;
    }

    /// How many sectors the whole array has.
    static uint8_t sector_count() {
        return static_cast<uint8_t>(sectors_per_bank() * bank_count());
    }

    /**
     * The `position`-th sector in ADDRESS order, 0 first. This is the
     * walk; sector() is the lookup by the manual's number, and the two
     * differ on a dual-bank part, where bank 2's first sector is
     * numbered 12 whatever bank 1's length (RM0090 tables 6 and 7).
     */
    static std::optional<FlashSector> sector_at(uint8_t position) {
        const uint8_t per_bank = sectors_per_bank();
        if (per_bank == 0u || position >= sector_count()) {
            return std::nullopt;
        }
        const uint8_t bank = static_cast<uint8_t>(position / per_bank);
        const uint8_t in_bank = static_cast<uint8_t>(position % per_bank);
        FlashSector s{};
        s.bank = static_cast<uint8_t>(bank + 1u);
        s.number = static_cast<uint8_t>(in_bank + (bank == 0u ? 0u : flash_bank2_first_sector));
        s.size = sector_size_in_bank(in_bank);
        uint32_t at = base + bank * bank_bytes();
        for (uint8_t i = 0; i < in_bank; ++i) {
            at += sector_size_in_bank(i);
        }
        s.base = at;
        return s;
    }

    /// The sector the manual calls `number`, or nothing when this part
    /// has no such sector.
    static std::optional<FlashSector> sector(uint8_t number) {
        for (uint8_t i = 0; i < sector_count(); ++i) {
            const std::optional<FlashSector> s = sector_at(i);
            if (s && s->number == number) {
                return s;
            }
        }
        return std::nullopt;
    }

    /// The sector holding `addr`.
    static std::optional<FlashSector> sector_of(uint32_t addr) {
        for (uint8_t i = 0; i < sector_count(); ++i) {
            const std::optional<FlashSector> s = sector_at(i);
            if (s && s->contains(addr)) {
                return s;
            }
        }
        return std::nullopt;
    }

    // ---- reading ------------------------------------------------------------

    /// Flash is memory-mapped: a read is a load, and a read of an erased
    /// cell returns 0xFF. Byte-wise on purpose - the caller's buffer has
    /// no alignment obligation.
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        const volatile uint8_t* src = reinterpret_cast<const volatile uint8_t*>(addr);
        for (uint32_t i = 0; i < dst.size(); ++i) {
            dst[i] = src[i];
        }
    }

    /// Whether every byte of a span of the array reads erased.
    static bool blank(uint32_t addr, uint32_t bytes) {
        const volatile uint8_t* at = reinterpret_cast<const volatile uint8_t*>(addr);
        for (uint32_t i = 0; i < bytes; ++i) {
            if (at[i] != 0xFFu) {
                return false;
            }
        }
        return true;
    }

    /// The OTP area (3.7): 512 bytes in sixteen blocks of 32, writable
    /// once and never erasable. There is no write verb here on purpose;
    /// this is the read.
    static bool read_otp(uint32_t offset, std::span<uint8_t> dst) {
        if (offset > flash_otp_bytes || dst.size() > flash_otp_bytes - offset) {
            return false;
        }
        read(flash_otp_base + offset, dst);
        return true;
    }

    /// Whether OTP block `i` (0..15) has been locked: its lock byte is
    /// programmed to 0x00 (3.7's table 11). "The lock bytes must only
    /// contain 0x00 and 0xFF values", so anything else is a locked
    /// block too - the safe reading.
    static bool otp_block_locked(uint8_t i) {
        if (i >= flash_otp_blocks) {
            return false;
        }
        return *reinterpret_cast<const volatile uint8_t*>(flash_otp_lock_base + i) != 0xFFu;
    }

    // ---- status -------------------------------------------------------------

    static uint32_t status() { return FLASH->SR; }
    static bool busy() { return (FLASH->SR & FlashFlag::busy) != 0u; }
    static uint32_t errors() { return FLASH->SR & FlashFlag::errors; }

    /// Write-one-to-clear, and only the bits that are.
    static void clear(uint32_t mask) { FLASH->SR = mask & FlashFlag::clearable; }
    static void clear_errors() { clear(FlashFlag::clearable); }

    /// The whole FLASH_SR the last operation ended on, EOP included -
    /// the flags this file's own return value drops. Diagnostic; no
    /// decision here consults it.
    static uint32_t last_status() { return last_sr_; }

    /**
     * How many turns the last OPERATION's completing wait spent, and
     * the one number that MEASURES the stall instead of assuming it: it
     * counts the loop iterations the CPU completed while the flash was
     * working. A core whose instruction fetches are stalled completes
     * none - which is what a single-bank part does with its code in the
     * array (3.5) - while a part reading the other bank, or running the
     * loop out of the instruction cache, completes millions.
     *
     * Only the operations publish it; the bookkeeping waits inside
     * unlock(), lock() and interrupts() do not, or they would wipe the
     * measurement the caller just made.
     */
    static uint32_t last_wait_turns() { return op_turns_; }

    /**
     * Wait until BSY falls. Bounded; false means it never did, and
     * every caller turns that into FlashFlag::refused rather than
     * proceed. 3.5.1's note is why no verb here writes FLASH_CR without
     * this first: "any attempt to write to it with the BSY bit set
     * causes the AHB bus to stall until the BSY bit is cleared" - a
     * stall this file would rather not enter blind.
     */
    static bool wait_ready() {
        for (uint32_t spins = 0; spins < wait_spins; ++spins) {
            if ((FLASH->SR & FlashFlag::busy) == 0u) {
                turns_ = spins;
                return true;
            }
        }
        turns_ = wait_spins;
        return false;
    }

    // ---- the lock -----------------------------------------------------------

    static bool locked() { return (FLASH->CR & FLASH_CR_LOCK) != 0u; }

    /**
     * KEY1 then KEY2 (3.5.1). Idempotent: an already-unlocked FLASH_CR
     * is left alone, because a key written into an unlocked KEYR is
     * itself the "wrong sequence" that returns a bus error and locks
     * the register until the next reset. That is why there is no
     * unconditional unlock verb in this file, and why nothing here
     * writes KEYR in any other order.
     */
    static bool unlock() {
        if (!locked()) {
            return true;
        }
        if (!wait_ready()) {
            return false;
        }
        FLASH->KEYR = key1;
        FLASH->KEYR = key2;
        return !locked();
    }

    /// Set LOCK again (a write-one bit; it cannot be cleared by hand).
    static bool lock() {
        if (!wait_ready()) {
            return false;
        }
        FLASH->CR = FLASH->CR | FLASH_CR_LOCK;
        return locked();
    }

    // ---- erase ---------------------------------------------------------------

    /**
     * Erase the sector the manual calls `number` (3.5.3): SER, SNB, then
     * STRT, then BSY.
     *
     * THE ADDRESS IS NOT AN ARGUMENT, on purpose. The sectors of this
     * family are uneven and their numbers are not contiguous across a
     * bank boundary, so an address rounded to "its" sector is exactly
     * the typo that erases a neighbour. A caller that has an address
     * asks sector_of() for the number and can see what it is about to
     * lose.
     *
     * The parallelism is the caller's claim about its supply (see
     * FlashParallelism) and it decides the erase TIME: 250 ms for a
     * 16 Kbyte sector at x32 against 400 ms at x8, 1 s for a 128 Kbyte
     * one against 2 s (DS10314 table 45).
     *
     * ON A SINGLE-BANK PART THIS FREEZES THE CORE for the whole
     * duration, interrupts included - last_wait_turns() is the witness.
     * Erasing the sector the code runs from is not refused here (no
     * driver can know where a caller's code will be when the pulse
     * lands) and is the one way to lose a board that this file leaves
     * open.
     */
    template <FlashParallelism p = FlashParallelism::x32>
    static uint32_t erase_sector(uint8_t number) {
        static_assert(p != FlashParallelism::x64,
                      "x64 needs 8..9 V on the VPP pad, which no board of this bench has: "
                      "erase at x32 (2.7 V and up), x16 or x8");
        const std::optional<FlashSector> s = sector(number);
        if (!s) {
            return FlashFlag::refused;
        }
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();
        const uint32_t cr = (FLASH->CR & keep_mask) | psize_bits(p) | FLASH_CR_SER |
                            (static_cast<uint32_t>(flash_sector_number_code(number)) << FLASH_CR_SNB_Pos);
        FLASH->CR = cr;
        FLASH->CR = cr | FLASH_CR_STRT;
        return finish();
    }

    /**
     * Erase one whole BANK - RM0090 3.6.3's "bank erase", the F42x/F43x
     * class's alone: MER for bank 1, the bit RM0090 calls MER1 and the
     * header spells MER2 for bank 2.
     *
     * The bank is a template argument so that naming bank 2 on a part
     * that has none is a compile error and not a silent store into a
     * reserved bit - which is exactly what the device header would let
     * happen, declaring `FLASH_CR_MER2` on the F446 too
     * (test/family_stm32f4/neg/flash_bank2_erase_on_a_single_bank.cpp).
     *
     * EXPOSED AND NEVER CALLED IN THIS TREE: bank 1 holds the running
     * image on every board here, so erasing it ends the program.
     *
     * The confirmation is a RUNTIME argument, where mass_erase()'s is a
     * template one: erasing a storage bank whole is a decision a program
     * may legitimately take while it runs, and erasing every user sector
     * is not.
     */
    template <FlashBank bank, FlashParallelism p = FlashParallelism::x32>
    static uint32_t bank_erase(FlashEraseAll confirm) {
        static_assert(p != FlashParallelism::x64, "x64 needs the VPP supply");
        static_assert(bank == FlashBank::bank1 || flash_facts().dual_bank_capable,
                      "this part class has one bank: RM0390 3.3 and RM0383 3.3 give it eight "
                      "sectors and no MER1, whatever FLASH_CR_MER2 the device header declares");
        if (confirm != FlashEraseAll::erase_every_user_sector) {
            return FlashFlag::refused;
        }
        if constexpr (bank == FlashBank::bank2) {
            if (bank_count() < 2u) {
                return FlashFlag::refused;
            }
        }
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();
        const uint32_t bit =
            bank == FlashBank::bank2 ? flash_bank2_mass_erase_mask() : FLASH_CR_MER;
        const uint32_t cr = (FLASH->CR & keep_mask) | psize_bits(p) | bit;
        FLASH->CR = cr;
        FLASH->CR = cr | FLASH_CR_STRT;
        return finish();
    }

    /**
     * Erase EVERY user sector (3.5.3's mass erase; on a dual-bank part
     * both mass-erase bits together, RM0090 3.6.3). The OTP area and the
     * configuration sector survive it.
     *
     * The confirmation is a template argument for the reason
     * FlashEraseAll gives: this verb takes down the running image, and
     * nothing in this tree calls it. It exists because the chapter has
     * it and a boot loader would want it.
     */
    template <FlashEraseAll confirm, FlashParallelism p = FlashParallelism::x32>
    static uint32_t mass_erase() {
        static_assert(p != FlashParallelism::x64, "x64 needs the VPP supply");
        static_assert(confirm == FlashEraseAll::erase_every_user_sector,
                      "mass_erase() erases every user sector, the running image included - "
                      "say FlashEraseAll::erase_every_user_sector or do not call it");
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();
        uint32_t bits = FLASH_CR_MER;
        if (bank_count() > 1u) {
            bits |= flash_bank2_mass_erase_mask();
        }
        const uint32_t cr = (FLASH->CR & keep_mask) | psize_bits(p) | bits;
        FLASH->CR = cr;
        FLASH->CR = cr | FLASH_CR_STRT;
        return finish();
    }

    // ---- program -------------------------------------------------------------

    /**
     * Program `src` at `addr` (3.5.4): PG, then one store per unit, each
     * followed by its wait.
     *
     * `addr` and the span's length are multiples of the parallelism's
     * unit - checked HERE, before the flash is touched, so a caller's
     * arithmetic slip costs a false and not a PGPERR storm. The stores
     * are volatile and of exactly the declared width: the width is the
     * one thing an optimizer is otherwise free to change, and changing
     * it is PGPERR.
     *
     * THE TARGET NEED NOT BE ERASED. 3.5.4: successive writes are
     * allowed while bits go from 1 to 0, and a 0 that has to become a 1
     * needs an erase - with no flag either way. So this verb cannot tell
     * a caller that its data did not land; read it back (blank() and
     * read() are here for that).
     *
     * Only the MAIN ARRAY is a legal target here. The OTP is reachable
     * by the same sequence and deliberately not offered (see the file
     * header); a store into the system memory or the configuration
     * sector raises WRPERR (3.6.4).
     */
    template <FlashParallelism p = FlashParallelism::x32>
    static uint32_t program(uint32_t addr, std::span<const uint8_t> src) {
        static_assert(p != FlashParallelism::x64,
                      "x64 needs 8..9 V on the VPP pad, which no board of this bench has: "
                      "program at x32 (2.7 V and up), x16 or x8");
        constexpr uint32_t unit = flash_parallelism_bytes(p);
        if (src.empty() || (addr % unit) != 0u || (src.size() % unit) != 0u) {
            return FlashFlag::refused;
        }
        if (!in_main_flash(addr) || src.size() > base + size_bytes() - addr) {
            return FlashFlag::refused;
        }
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();
        FLASH->CR = (FLASH->CR & keep_mask) | psize_bits(p) | FLASH_CR_PG;
        uint32_t err = 0;
        for (uint32_t off = 0; off < src.size(); off += unit) {
            store<p>(addr + off, src.data() + off);
            if (!wait_ready()) {
                op_turns_ = turns_;
                err |= FlashFlag::refused;
                break;
            }
            op_turns_ = turns_;
            last_sr_ = FLASH->SR;
            clear(last_sr_);
            err |= last_sr_ & FlashFlag::errors;
            if (err != 0u) {
                break;
            }
        }
        release_cr();
        return err;
    }

    /// One 32-bit word at x32 - the shape most callers want, with the
    /// source in a register instead of a buffer.
    static uint32_t program_word(uint32_t addr, uint32_t value) {
        const uint8_t bytes[4] = {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
                                  static_cast<uint8_t>(value >> 16),
                                  static_cast<uint8_t>(value >> 24)};
        return program<FlashParallelism::x32>(addr, std::span<const uint8_t>{bytes, 4});
    }

    // ---- the three malformed sequences, on purpose ---------------------------

    /**
     * 3.5.4 names three ways to get a program wrong and gives each a
     * flag; 3.6.4 adds a fourth for a target that may not be written.
     * Everything else in this file REFUSES those before the flash is
     * touched - which is right for an application and useless for a
     * bench suite, whose job is to see the SILICON raise PGSERR, PGPERR
     * and WRPERR rather than to see this file's bounds check.
     *
     * So they are spelled here, once, deliberately named, and nothing
     * but a suite has any business calling them. NONE OF THEM WRITES:
     * each is aborted by the silicon, which is why the letter that
     * stages them costs no endurance. `addr` must be a word-aligned
     * address in a sector the caller owns.
     */
    enum class Misstep : uint8_t {
        store_without_pg,      ///< PGSERR: a store into the array with PG clear
        wrong_access_width,    ///< PGPERR: a byte store while PSIZE says x32
        unaligned_word,        ///< a word store off its alignment (the flag is measured)
        write_protected_area,  ///< WRPERR: a store into the system memory
    };

    static uint32_t provoke(Misstep step, uint32_t addr) {
        if (!in_main_flash(addr) || (addr % 4u) != 0u) {
            return FlashFlag::refused;
        }
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();
        if (step != Misstep::store_without_pg) {
            FLASH->CR = (FLASH->CR & keep_mask) | psize_bits(FlashParallelism::x32) | FLASH_CR_PG;
        }
        switch (step) {
            case Misstep::store_without_pg:
                *reinterpret_cast<volatile uint32_t*>(addr) = 0x1234'5678UL;
                break;
            case Misstep::wrong_access_width:
                // One BYTE with PSIZE declaring x32 - 3.5.4's "the write
                // access type must correspond to the type of parallelism
                // chosen. If not, the write operation is not performed".
                *reinterpret_cast<volatile uint8_t*>(addr) = 0x5Au;
                break;
            case Misstep::unaligned_word:
                // A word store two bytes off its alignment. The core
                // splits it (UNALIGN_TRP is clear out of reset), so what
                // reaches the engine is not one word - which flag that
                // earns is the suite's measurement, not this file's
                // claim.
                *reinterpret_cast<volatile uint32_t*>(addr + 2u) = 0x1234'5678UL;
                break;
            case Misstep::write_protected_area:
                // The system memory: read-only to user code for ever,
                // and 3.6.4's first bullet for WRPERR.
                *reinterpret_cast<volatile uint32_t*>(flash_system_memory_base) = 0x1234'5678UL;
                break;
        }
        return finish();
    }

    /**
     * The misstep that is not a program at all, and the only one in this
     * file with a lasting consequence: ONE wrong word into FLASH_KEYR.
     *
     * 3.5.1: "Any wrong sequence returns a bus error and lock up the
     * FLASH_CR register until the next reset." Both halves of that
     * sentence are why this is a verb of its own rather than a Misstep -
     * the caller must ARM A FAULT HANDLER first (the store itself is the
     * bus error, and on this core it is imprecise, so it arrives as a
     * BusFault or, with that vector disabled, as a HardFault), and after
     * it the engine is dead: unlock() cannot revive it, every erase and
     * program refuses, and only a reset brings the interface back.
     *
     * It exists so that a bench suite can MEASURE that lockout once,
     * last, with a re-flash behind it. Nothing else has any business
     * calling it. The key written is the complement of KEY1, which is a
     * wrong first word by any reading of the sequence.
     */
    static void provoke_wrong_key() {
        if (!locked()) {
            lock();
        }
        FLASH->KEYR = ~key1;
        __DSB();
    }

    // ---- the interrupt --------------------------------------------------------

    static constexpr IRQn_Type irq() { return flash_irq(); }

    /**
     * EOPIE and ERRIE (3.5.5). Note what they really are: 3.8.4 says EOP
     * and OPERR are each SET ONLY IF its enable bit is set, so these are
     * not merely interrupt masks - they decide whether the flag exists
     * at all. That is why nothing in this file waits on EOP.
     */
    static bool interrupts(bool end_of_operation, bool error) {
        if (!wait_ready()) {
            return false;
        }
        uint32_t cr = FLASH->CR & ~(FLASH_CR_EOPIE | FLASH_CR_ERRIE);
        if (end_of_operation) {
            cr |= FLASH_CR_EOPIE;
        }
        if (error) {
            cr |= FLASH_CR_ERRIE;
        }
        FLASH->CR = cr;
        return true;
    }

    static bool end_of_operation_interrupt() { return (FLASH->CR & FLASH_CR_EOPIE) != 0u; }
    static bool error_interrupt() { return (FLASH->CR & FLASH_CR_ERRIE) != 0u; }

    /**
     * The FLASH_IRQHandler body an app binds to the vector. Returns the
     * flags it found and clears exactly those, so a handler can report
     * without a second read racing the next operation. The line is a
     * LEVEL and stays asserted while any of them stands, which is why
     * clearing is the body's first and only duty - and why a handler
     * that only reads would be entered for ever.
     *
     * One entry per error, measured: the clearing store lands before the
     * handler returns and the level does not re-latch. What DOES latch
     * is an error raised while the line is masked at the NVIC - the
     * pending bit is set all the same, and the handler runs on history
     * the moment the line is opened. A program that masks the line takes
     * the pending bit down before opening it again.
     */
    [[gnu::always_inline]] static inline uint32_t isr() {
        const uint32_t sr = FLASH->SR & FlashFlag::clearable;
        if (sr != 0u) {
            FLASH->SR = sr;
        }
        return sr;
    }

private:
    /// The bits of FLASH_CR an operation must clear before it sets its
    /// own: 3.5.3's note makes SER together with a mass-erase bit a mass
    /// erase, and a stale SNB would aim the next one at the wrong
    /// sector. PSIZE is rewritten by every operation, so it is dropped
    /// here too.
    static constexpr uint32_t snb_mask =
        ((1UL << flash_facts().snb_bits) - 1UL) << FLASH_CR_SNB_Pos;
    static constexpr uint32_t keep_mask = ~(FLASH_CR_PG | FLASH_CR_SER | FLASH_CR_MER |
                                            flash_bank2_mass_erase_mask() | snb_mask |
                                            FLASH_CR_PSIZE_Msk);

    static constexpr uint32_t psize_bits(FlashParallelism p) {
        return static_cast<uint32_t>(p) << FLASH_CR_PSIZE_Pos;
    }

    /// The store, at exactly the declared width.
    template <FlashParallelism p>
    static void store(uint32_t addr, const uint8_t* src) {
        if constexpr (p == FlashParallelism::x8) {
            *reinterpret_cast<volatile uint8_t*>(addr) = src[0];
        } else if constexpr (p == FlashParallelism::x16) {
            *reinterpret_cast<volatile uint16_t*>(addr) =
                static_cast<uint16_t>(static_cast<uint16_t>(src[0]) |
                                      static_cast<uint16_t>(static_cast<uint16_t>(src[1]) << 8));
        } else {
            *reinterpret_cast<volatile uint32_t*>(addr) =
                static_cast<uint32_t>(src[0]) | (static_cast<uint32_t>(src[1]) << 8) |
                (static_cast<uint32_t>(src[2]) << 16) | (static_cast<uint32_t>(src[3]) << 24);
        }
    }

    /// The tail every started operation shares: wait, bank the evidence,
    /// put FLASH_CR back where the next one is legal, clear what was
    /// seen, and answer with the errors alone.
    static uint32_t finish() {
        const bool done = wait_ready();
        op_turns_ = turns_;
        last_sr_ = FLASH->SR;
        release_cr();
        clear(last_sr_);
        return done ? (last_sr_ & FlashFlag::errors) : FlashFlag::refused;
    }

    static void release_cr() { FLASH->CR = FLASH->CR & keep_mask; }

    static inline uint32_t last_sr_ = 0;
    /// Every wait's own count; only an operation copies it out.
    static inline uint32_t turns_ = 0;
    static inline uint32_t op_turns_ = 0;
};

// ---- the option bytes --------------------------------------------------------

/// RDP as 3.6.3 codes it: 0xAA is level 0, 0xCC is level 2, ANYTHING
/// ELSE is level 1. There is no fourth possibility, which is why the
/// decode is a function and not a table.
enum class FlashRdpLevel : uint8_t { level0 = 0, level1 = 1, level2 = 2 };

/**
 * FLASH_OPTCR and FLASH_OPTCR1 (3.6): the user option bytes, DECODED -
 * and, for the write protection alone, written.
 *
 * WHAT IS READ AND NEVER WRITTEN: RDP, the BOR level, WDG_SW, the two
 * nRST_* bits, SPRMOD, BFB2 and DB1M. Every one of them changes how the
 * part BOOTS, and none of them belongs to the running program: the
 * watchdog mode needs a system reset to take effect (3.8.6's note), the
 * dual-bank bits renumber the sectors under the erase engine, and RDP
 * level 2 is irreversible and takes the debug port with it.
 * Provisioning is a tool's job over the debug port; what firmware needs
 * from these bytes is to KNOW.
 *
 * WHAT IS WRITTEN: nWRP, one bit per sector, through protect(). It is
 * the one option a running program has a use for (a boot loader
 * protecting itself), it is reversible at RDP level 0 with PCROP off -
 * figure 4's "options write (RDP level identical)" is an option erase
 * and a reprogram, and it stays at level 0 - and protect() refuses in
 * any other state. THE RDP BYTE IS NOT IN THE DATA PATH: the write goes
 * through a HALF-WORD store to OPTCR's upper half, which carries nWRP
 * and SPRMOD and nothing else, and the start bit through a BYTE store
 * to its lowest.
 *
 * EVERY OPTION WRITE ERASES AND REPROGRAMS THE WHOLE CONFIGURATION
 * SECTOR (3.6.2's note), so it costs that sector an endurance cycle -
 * and the sector is the one that decides whether the part boots.
 */
struct FlashOptions {
    FlashOptions() = delete;

    static uint32_t raw() { return FLASH->OPTCR; }

    /// OPTCR1 - bank 2's write protection, and only where a bank 2
    /// exists. ST declares the register on every part of the pack, so
    /// the reserve's fact and not the struct member is what decides:
    /// zero where there is none.
    static uint32_t raw_bank2() {
        return flash_facts().has_option_register1 ? FLASH->OPTCR1 : 0u;
    }

    static uint8_t rdp_code() {
        return static_cast<uint8_t>((FLASH->OPTCR & FLASH_OPTCR_RDP_Msk) >> FLASH_OPTCR_RDP_Pos);
    }
    static FlashRdpLevel rdp() {
        const uint8_t c = rdp_code();
        if (c == 0xAAu) {
            return FlashRdpLevel::level0;
        }
        if (c == 0xCCu) {
            return FlashRdpLevel::level2;
        }
        return FlashRdpLevel::level1;
    }

    /// BOR_LEV, as 3.8.6 codes it - and the code is NOT the manual's
    /// level number: 0 is "BOR Level 3", 1 is Level 2, 2 is Level 1 and
    /// 3 is BOR OFF, which is why bor_off() exists beside the raw
    /// field. The volts behind each level are the datasheet's and are
    /// not carried here.
    static uint8_t bor_level() {
        return static_cast<uint8_t>((FLASH->OPTCR & FLASH_OPTCR_BOR_LEV_Msk) >>
                                    FLASH_OPTCR_BOR_LEV_Pos);
    }
    static bool bor_off() { return bor_level() == 3u; }

    /// WDG_SW = 1 is the SOFTWARE watchdog: the IWDG is started by
    /// firmware. 0 is the hardware one, which runs from the boot -
    /// stm32f4/reset.hpp's `Iwdg` has no say in this, and a program
    /// that finds a 0 here was started with its watchdog already
    /// running.
    static bool iwdg_software() { return (FLASH->OPTCR & FLASH_OPTCR_WDG_SW) != 0u; }

    // The two nRST_* bits are spelled here the way they READ, not the
    // way they are named: the option bit is 1 for "no reset", so a verb
    // called reset_on_stop() has to be its complement or the name lies.
    static bool reset_on_stop() { return (FLASH->OPTCR & FLASH_OPTCR_nRST_STOP) == 0u; }
    static bool reset_on_standby() { return (FLASH->OPTCR & FLASH_OPTCR_nRST_STDBY) == 0u; }

    /// SPRMOD: the nWRP bits mean PCROP instead of write protection
    /// (3.6.5). Always false where the part class has no PCROP.
    static constexpr bool has_pcrop = flash_facts().has_pcrop_mode;
    static bool pcrop_mode() {
        return has_pcrop && (FLASH->OPTCR & flash_pcrop_mode_mask()) != 0u;
    }

    /// BFB2 and DB1M, the two boot-and-bank options of the F42x/F43x
    /// class; false where the class has neither.
    static bool dual_bank_boot() {
        return flash_facts().has_dual_bank_boot &&
               (FLASH->OPTCR & flash_dual_bank_boot_mask()) != 0u;
    }
    static bool dual_bank_1m() {
        return flash_facts().has_dual_bank_option &&
               (FLASH->OPTCR & flash_dual_bank_option_mask()) != 0u;
    }

    /// The nWRP bit of sector `number` as it stands: true = the bit is
    /// SET, which means "not write protected" with SPRMOD clear and
    /// "PCROP protected" with it set (3.8.6). The two readings are
    /// write_protected() and pcrop_protected().
    static std::optional<bool> nwrp(uint8_t number) {
        const std::optional<uint32_t> b = nwrp_bit(number);
        if (!b) {
            return std::nullopt;
        }
        const uint32_t reg = number >= flash_bank2_first_sector ? raw_bank2() : raw();
        return (reg & *b) != 0u;
    }

    static bool write_protected(uint8_t number) {
        const std::optional<bool> set = nwrp(number);
        return set && !*set && !pcrop_mode();
    }

    static bool pcrop_protected(uint8_t number) {
        const std::optional<bool> set = nwrp(number);
        return set && *set && pcrop_mode();
    }

    // ---- the option engine ---------------------------------------------------

    static bool locked() { return (FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) != 0u; }

    /// OPTKEY1 then OPTKEY2 (3.6.2), with the same idempotence and the
    /// same reason as Flash::unlock(): a key written into an unlocked
    /// register is the wrong sequence, and the wrong sequence locks
    /// OPTCR until the next reset.
    static bool unlock() {
        if (!locked()) {
            return true;
        }
        if (!Flash::wait_ready()) {
            return false;
        }
        FLASH->OPTKEYR = Flash::option_key1;
        FLASH->OPTKEYR = Flash::option_key2;
        return !locked();
    }

    static bool lock() {
        if (!Flash::wait_ready()) {
            return false;
        }
        FLASH->OPTCR = FLASH->OPTCR | FLASH_OPTCR_OPTLOCK;
        return locked();
    }

    /**
     * Write-protect sector `number`, or take the protection off
     * (3.6.4). The option engine must be unlocked first.
     *
     * REFUSED unless RDP reads 0xAA and PCROP is off - the state figure
     * 4 makes an option write reversible in, and the one in which the
     * nWRP bits mean write protection at all. Refused too for a sector
     * this part has not got, and for a bank-2 sector where OPTCR1 does
     * not exist.
     *
     * The RDP byte is not written: the new nWRP goes in through a
     * HALF-WORD store to the upper half of OPTCR (or OPTCR1), which
     * holds nWRP and SPRMOD alone, and OPTSTRT through a BYTE store to
     * the lowest, which holds the BOR level, the three user bits and
     * the two control bits and is written back exactly as it read.
     *
     * True when the option loader has taken the new value: the verb
     * waits for BSY and then reads the bit back. What it costs is one
     * endurance cycle of the configuration sector.
     */
    static bool protect(uint8_t number, bool on) {
        const std::optional<uint32_t> b = nwrp_bit(number);
        if (!b) {
            return false;
        }
        if (rdp_code() != 0xAAu || pcrop_mode() || locked()) {
            return false;
        }
        if (!Flash::wait_ready()) {
            return false;
        }
        const bool bank2 = number >= flash_bank2_first_sector;
        volatile uint16_t* const high = reinterpret_cast<volatile uint16_t*>(
            (bank2 ? reinterpret_cast<uintptr_t>(&FLASH->OPTCR1)
                   : reinterpret_cast<uintptr_t>(&FLASH->OPTCR)) +
            2u);
        const uint16_t bit = static_cast<uint16_t>(*b >> 16);
        const uint16_t before = *high;
        // nWRP is ACTIVE LOW: a zero protects.
        *high = static_cast<uint16_t>(on ? (before & static_cast<uint16_t>(~bit))
                                         : (before | bit));
        volatile uint8_t* const low = reinterpret_cast<volatile uint8_t*>(&FLASH->OPTCR);
        *low = static_cast<uint8_t>(*low | FLASH_OPTCR_OPTSTRT);
        if (!Flash::wait_ready()) {
            return false;
        }
        const std::optional<bool> now = nwrp(number);
        return now && (*now != on);
    }

private:
    /// Which bit of which OPTCR carries sector `number`, or nothing when
    /// there is no such bit on this part.
    static std::optional<uint32_t> nwrp_bit(uint8_t number) {
        const FlashFacts f = flash_facts();
        if (!f.known || !Flash::sector(number)) {
            return std::nullopt;
        }
        const bool bank2 = number >= flash_bank2_first_sector;
        if (bank2 && !f.has_option_register1) {
            return std::nullopt;
        }
        const uint8_t in_bank =
            static_cast<uint8_t>(bank2 ? number - flash_bank2_first_sector : number);
        if (in_bank >= f.wrp_bits) {
            return std::nullopt;
        }
        return 1UL << (FLASH_OPTCR_nWRP_Pos + in_bank);
    }
};

} // namespace brio
