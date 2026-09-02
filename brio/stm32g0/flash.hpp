/*
 * flash.hpp
 *
 * The embedded flash memory and its interface (RM0444 ch. 3): the read
 * access latency the clock task must set before it raises HCLK, the two
 * CPU-side accelerators, and - since the FLASH campaign - the whole of
 * the program/erase engine, the option bytes as a READ-ONLY decode, the
 * ECC status and the one interrupt.
 *
 * WHY THE WAIT STATES LIVE HERE AND NOT IN clock.hpp: they are the flash
 * interface's register, and the samc stratum paid for the other choice
 * (clock.hpp squatted on NVMCTRL's RWS until the NVM campaign took it
 * back). The clock task calls in; this file owns the register.
 *
 * WHAT THE CHAPTER IS, in the shape the code takes:
 *
 *  - ONE ARRAY, TWO BANKS, THREE GRANULARITIES. A page is 2 Kbytes and
 *    is the erase unit; a row is 256 bytes and is the FAST programming
 *    unit; a DOUBLE WORD of 8 bytes is the ordinary program unit and the
 *    ECC unit (64 data bits plus 8 ECC bits, 3.3.1). Nothing smaller can
 *    be written at all: a byte or half-word store raises SIZERR, and a
 *    double word that is not double-word aligned raises PGAERR (3.3.8).
 *
 *  - A CELL IS WRITTEN ONCE BETWEEN ERASES. 3.3.8: "Programming a
 *    previously programmed address with a non-zero data is not allowed"
 *    and sets PROGERR. That is exactly util/nv_heap.hpp's write_cell
 *    contract, which is why the two fit with nothing in between.
 *
 *  - THE UNLOCK IS A KEYED SEQUENCE AND A WRONG ONE IS FATAL UNTIL THE
 *    NEXT RESET. 3.3.6: KEY1 then KEY2 into FLASH_KEYR; "any wrong
 *    sequence locks the FLASH_CR register until the next system reset",
 *    and the wrong write is itself a bus error, i.e. a HardFault. So
 *    unlock() writes the pair only when LOCK really stands, and there is
 *    no verb anywhere in this file that can produce a half sequence.
 *
 *  - FLASH_CR MUST NOT BE WRITTEN WHILE CFGBSY STANDS: 3.7.4 says such a
 *    write "causes HardFault". Every verb here waits first, and the wait
 *    is BOUNDED - a spin count, not a while(1) - because the alternative
 *    on a sick chip is a mute board.
 *
 *  - EOP IS NOT A COMPLETION WITNESS. 3.7.4 bit 0: "This bit is set only
 *    if the end of operation interrupts are enabled (EOPIE=1)", which
 *    contradicts 3.3.8's own step 7 telling the programmer to check it.
 *    This file therefore judges an operation by CFGBSY falling and by
 *    the ERROR bits, and EOP is reported but never required. (The bench
 *    settles which document is right - docs/stm32g0/nvm.md.)
 *
 *  - HSI16 IS TURNED ON BY THE ENGINE ITSELF when PG, FSTPG or STRT is
 *    set, and turned off again unless RCC_CR.HSION was already set
 *    (3.3.7, 3.3.8). Nothing here manages it: on a brio board the PLL
 *    already runs from HSI16, and on one that does not the engine's own
 *    request is the documented behaviour.
 *
 *  - READ-WHILE-WRITE (3.3.9): on a dual-bank part, a program or erase
 *    in one bank leaves reads of the OTHER bank running. In the same
 *    bank the read stalls the bus (3.3.6). That single sentence is what
 *    makes stm32g0/nvm_flash.hpp put all storage in bank 2 and all code
 *    in bank 1, and it is the property the bench letter measures.
 *
 *  - THE OPTION BYTES ARE READ AND NEVER WRITTEN. There is no
 *    OPTKEYR verb, no OPTSTRT verb and no OBL_LAUNCH verb in this file,
 *    on purpose: RDP Level 2 is irreversible, and ES0548 2.2.9 says an
 *    option-byte MISMATCH can leave the device with BOOT_LOCK set and
 *    the debug interface gone - a brick, from one interrupted write.
 *    Provisioning belongs to a tool over SWD, the way fuses do on the
 *    other two targets. FlashOptions decodes what is there.
 *
 *  - THE OTP AREA (3.3.1) is 1 Kbyte of double words that can be written
 *    ONCE and never erased - not even back to zero. It is memory-mapped
 *    and readable like any other flash; this file exposes the READ and
 *    deliberately no write.
 *
 * ERRATA that touch this chapter, ES0548 Rev 3, silicon revision Z:
 *  - 2.2.3, LIVE: a location holding all ones cannot be re-programmed to
 *    all zeros - the one exception 3.3.8 grants to the write-once rule
 *    does not work. No workaround. It is UNREACHABLE BY CONSTRUCTION for
 *    everything built on top of this file, because util/nv_heap.hpp and
 *    util/nv_journal.hpp never program a cell twice between erases; a
 *    caller that wants that exception is told here that it does not
 *    exist.
 *  - 2.2.10, LIVE: prefetch may fail on a branch across banks, with no
 *    workaround, and the erratum's own note blesses "EEPROM emulation or
 *    other data storage in bank 2" as the safe use of dual bank. That is
 *    precisely nvm_flash.hpp's design, and it is also why FlashAccel
 *    leaves PRFTEN at its reset value (clear).
 *  - 2.2.5 (PCROP read weakness) is rev A only and nothing here sets
 *    PCROP anyway; 2.2.9 (option-byte mismatch) is answered by not
 *    writing option bytes at all.
 *
 * Facts that shape the wait-state code (RM0444 3.3.4, 3.7.1):
 *  - table 13: at VCORE Range 1 (the reset range, and the only one this
 *    stratum runs in) HCLK <= 24 MHz needs 0 wait states, <= 48 needs 1,
 *    <= 64 needs 2; Range 2 halves the ceilings (8 / 16 MHz) and forbids
 *    2 WS. Out of reset HCLK is 16 MHz at 0 WS.
 *  - the ORDER is the same rule as on every target: wait states go UP
 *    before a frequency rise and DOWN after a fall, and a new LATENCY
 *    value is in force only when it READS BACK - 3.7.1 says so in one
 *    sentence, and the samc side proved the cost of not waiting.
 *  - ICEN (instruction cache) is set at reset, PRFTEN (prefetch) is
 *    clear. This stratum leaves both at their reset values: erratum
 *    ES0548 2.2.10 (see above) makes PRFTEN a decision to take
 *    knowingly, with a measurement, not a default.
 */

#pragma once

#include <stdint.h>

#include <span>

#include "stm32g0/device_tables.hpp"
#include "stm32g0xx.h"

namespace brio {

/// FLASH_ACR.LATENCY - the read wait states, and the rule that picks
/// them. Monostate: one flash interface per device.
struct FlashWaitStates {
    FlashWaitStates() = delete;

    static constexpr uint8_t max_latency = 2;

    static uint8_t get() {
        return static_cast<uint8_t>(FLASH->ACR & FLASH_ACR_LATENCY_Msk);
    }

    /// Program `ws` and wait until it reads back (3.7.1: the write
    /// becomes effective when it returns the same value upon read).
    /// Bounded: a value the field cannot take (> 2) would never read
    /// back, so it is refused instead, and false says so.
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

    /// Table 13, VCORE Range 1 column: the wait states HCLK needs.
    static constexpr uint8_t for_hz(uint32_t hz) {
        if (hz <= 24'000'000UL) return 0;
        if (hz <= 48'000'000UL) return 1;
        return 2;
    }

    /// Table 13, Range 2 column - declared for the day a low-power
    /// clock task runs the core in Range 2; nothing calls it yet.
    static constexpr uint8_t for_hz_range2(uint32_t hz) {
        return hz <= 8'000'000UL ? 0 : 1;
    }
};

/// The two CPU-side accelerators in FLASH_ACR. Readback verbs only need
/// a plain load; the setters are plain stores (no synchronization).
struct FlashAccel {
    FlashAccel() = delete;

    static bool prefetch() { return (FLASH->ACR & FLASH_ACR_PRFTEN) != 0u; }
    static void prefetch(bool on) {
        FLASH->ACR = on ? (FLASH->ACR | FLASH_ACR_PRFTEN) : (FLASH->ACR & ~FLASH_ACR_PRFTEN);
    }

    static bool instruction_cache() { return (FLASH->ACR & FLASH_ACR_ICEN) != 0u; }
    static void instruction_cache(bool on) {
        FLASH->ACR = on ? (FLASH->ACR | FLASH_ACR_ICEN) : (FLASH->ACR & ~FLASH_ACR_ICEN);
    }

    /// Flush the instruction cache. 3.3.8 is explicit that this is legal
    /// ONLY with the cache disabled, so the verb does the whole dance -
    /// off, reset, on again - and restores what it found. An erase that
    /// took down cached code needs it (3.3.8, "Programming and cache").
    static void flush_instruction_cache() {
        const bool was_on = instruction_cache();
        instruction_cache(false);
        FLASH->ACR |= FLASH_ACR_ICRST;
        FLASH->ACR &= ~FLASH_ACR_ICRST;
        instruction_cache(was_on);
    }

    /// FLASH_ACR.PROGEMPTY, the empty-check bit the boot loader reads:
    /// 1 = the first location of the user flash is erased. Read-only
    /// here (writing it is a boot-configuration decision).
    static bool user_flash_empty() {
        return (FLASH->ACR & FLASH_ACR_PROGEMPTY) != 0u;
    }

    /// FLASH_ACR.DBG_SWEN - whether the debug port is allowed to reach
    /// the core. Read-only here: clearing it is one of the two ways to
    /// lose a board (the other is RDP 2), and 3.5.5 is where the story
    /// is told.
    static bool debug_access() { return (FLASH->ACR & FLASH_ACR_DBG_SWEN) != 0u; }
};

/// The flash size the part reports, in kilobytes (RM0444 41.2: the
/// register holds it in KB; the header's FLASH_SIZE macro is the same
/// read in bytes).
inline uint32_t flash_size_kb() {
    return *reinterpret_cast<const volatile uint32_t*>(FLASHSIZE_BASE) & 0x03FFu;
}

/// Which PHYSICAL bank an erase acts on. 3.3.2: erase is always linked
/// to a physical bank and is NOT affected by nSWAP_BANK, while
/// programming follows the logical address - so this enum is only ever
/// needed on the erase path, and Flash::erase_bank_of() is what turns a
/// logical address into it.
enum class FlashBank : uint8_t { bank1 = 0, bank2 = 1 };

/**
 * FLASH_SR, bit by bit, as one mask vocabulary.
 *
 * Every operation in this file returns a MASK of these rather than an
 * enumerated single cause, because several can stand at once: 3.3.8 says
 * PGSERR is set whenever any other error is, and FASTERR is set beside
 * whichever error interrupted a fast row. An enum would have to choose
 * one and would therefore lose the evidence.
 *
 * `refused` is NOT a silicon bit. Bits 31:19 of FLASH_SR are Reserved,
 * so this file borrows the top one to mean "this driver refused before
 * the flash was touched" - an address that is not aligned, a locked
 * FLASH_CR, a wait that timed out. It can never collide with a real
 * flag, and a caller that only wants the hardware's opinion masks it
 * off with `errors`.
 */
struct FlashFlag {
    FlashFlag() = delete;

    // Write-one-to-clear status.
    static constexpr uint32_t eop = FLASH_SR_EOP;
    static constexpr uint32_t operation_error = FLASH_SR_OPERR;
    static constexpr uint32_t program_error = FLASH_SR_PROGERR;
    static constexpr uint32_t write_protect_error = FLASH_SR_WRPERR;
    static constexpr uint32_t alignment_error = FLASH_SR_PGAERR;
    static constexpr uint32_t size_error = FLASH_SR_SIZERR;
    static constexpr uint32_t sequence_error = FLASH_SR_PGSERR;
    /// RM0444 3.7.4 calls this MISSERR; the device header spells the
    /// mask FLASH_SR_MISERR. Same bit 8, and the header is what compiles.
    static constexpr uint32_t miss_error = FLASH_SR_MISERR;
    static constexpr uint32_t fast_error = FLASH_SR_FASTERR;
    static constexpr uint32_t read_protect_error = FLASH_SR_RDERR;
    static constexpr uint32_t option_error = FLASH_SR_OPTVERR;

    // Read-only status.
    static constexpr uint32_t bank1_busy = FLASH_SR_BSY1;
    static constexpr uint32_t bank2_busy = flash_sr_bank2_busy;
    static constexpr uint32_t config_busy = FLASH_SR_CFGBSY;

    /// This file's own: nothing was written to the flash at all.
    static constexpr uint32_t refused = 0x8000'0000UL;

    /// Everything that says an operation went wrong.
    static constexpr uint32_t errors =
        operation_error | program_error | write_protect_error | alignment_error |
        size_error | sequence_error | miss_error | fast_error |
        read_protect_error | option_error;

    /// Everything a store of ones takes down.
    static constexpr uint32_t clearable = errors | eop;

    static constexpr uint32_t busy = bank1_busy | bank2_busy;
};

/// What FLASH_ECCR (or FLASH_ECCR2) is saying, decoded.
struct FlashEccStatus {
    bool corrected = false;      ///< ECCC: one bit was wrong and was fixed
    bool detected = false;       ///< ECCD: two bits were wrong - an NMI was raised
    bool system_flash = false;   ///< SYSF_ECC: the failure is in system memory
    bool present = false;        ///< false when this bank does not exist
    uint32_t double_word_offset = 0;  ///< ADDR_ECC, in 64-bit units
    uint32_t raw = 0;
};

/**
 * The program and erase engine (RM0444 3.3.6 .. 3.3.9).
 *
 * Monostate: there is one flash interface and it is a piece of hardware.
 * Every verb that touches FLASH_CR waits for CFGBSY first, because
 * 3.7.5 says writing that register while CFGBSY stands "causes a
 * HardFault exception" - so on this target a missing wait is not a lost
 * store but a dead board.
 *
 * THE RETURN VALUE IS A MASK, zero meaning success (FlashFlag). The mask
 * is also cleared out of FLASH_SR on the way out, so a later operation
 * cannot trip over an old error - 3.3.7 and 3.3.8 both begin with "check
 * and clear all error flags due to a previous programming. If not,
 * PGSERR is set", and this file does that as its own first act.
 *
 * WHAT IS NOT HERE: nothing writes an option byte, and nothing writes
 * the OTP. See the file header for why.
 */
struct Flash {
    Flash() = delete;

    // ---- geometry, the family's constants ----------------------------------
    //
    // These are the same on every STM32G0 (3.2, 3.3.1); what varies is
    // how many pages and how many banks a part has, which is the flash
    // size register's business and therefore runtime.

    static constexpr uint32_t base = FLASH_BASE;
    static constexpr uint32_t page_size = 2048;      ///< the ERASE unit
    static constexpr uint32_t subpage_size = 512;    ///< the PCROP unit
    static constexpr uint32_t row_size = 256;        ///< the FAST program unit
    static constexpr uint32_t cell_size = 8;         ///< a double word: program + ECC unit
    static constexpr uint32_t cells_per_row = row_size / cell_size;   // 32
    static constexpr uint32_t max_pages_per_bank = 128;

    static constexpr uint32_t otp_base = 0x1FFF'7000UL;
    static constexpr uint32_t otp_size = 1024;
    static constexpr uint32_t option_bytes_base = 0x1FFF'7800UL;

    static constexpr uint32_t key1 = 0x4567'0123UL;
    static constexpr uint32_t key2 = 0xCDEF'89ABUL;

    /// The bound on every wait. A page erase is up to 40 ms and a mass
    /// erase up to 40.1 ms (DS13560 table 48); at 64 MHz that is under
    /// 2.6 million core cycles, and one turn of the loop below is
    /// several of them. Four million turns is therefore many times the
    /// worst case and still finite, which is the whole point: a sick
    /// chip must give the caller a false, not a mute board.
    static constexpr uint32_t wait_spins = 4'000'000UL;

    // ---- what this particular part is --------------------------------------

    static uint32_t size_bytes() { return flash_size_kb() * 1024u; }
    static uint32_t page_count() { return size_bytes() / page_size; }

    /// 3.3.2 and table 12: a 512 KB part is ALWAYS dual-bank whatever the
    /// option bit says, a 128 KB part never is, and only the 256 KB one
    /// is ruled by DUAL_BANK.
    static uint8_t bank_count() {
        if (!flash_dual_bank_capable) {
            return 1;
        }
        const uint32_t kb = flash_size_kb();
        if (kb > 256u) {
            return 2;
        }
        if (kb <= 128u) {
            return 1;
        }
        return (FLASH->OPTR & flash_optr_dual_bank) != 0u ? 2 : 1;
    }

    static uint32_t bank_size() { return size_bytes() / bank_count(); }
    static uint32_t pages_per_bank() { return bank_size() / page_size; }

    /// True when the physical banks are exchanged in the address map -
    /// nSWAP_BANK reads 0 (3.7.8). Always false where the bit does not
    /// exist.
    static bool banks_swapped() {
        return flash_optr_swap_bank != 0u &&
               (FLASH->OPTR & flash_optr_swap_bank) == 0u;
    }

    /// The PHYSICAL bank holding the logical address `addr`, which is
    /// what FLASH_CR.BKER wants. 3.3.2: erase is bound to the physical
    /// bank and ignores the swap, so this is the one place in the file
    /// where nSWAP_BANK changes an outcome.
    static FlashBank erase_bank_of(uint32_t addr) {
        if (bank_count() < 2) {
            return FlashBank::bank1;
        }
        const bool upper = (addr - base) >= bank_size();
        return (upper != banks_swapped()) ? FlashBank::bank2 : FlashBank::bank1;
    }

    /// FLASH_CR.PNB for `addr`: the page's index WITHIN ITS BANK.
    static uint32_t page_of(uint32_t addr) {
        return ((addr - base) % bank_size()) / page_size;
    }

    static bool in_main_flash(uint32_t addr) {
        return addr >= base && addr < base + size_bytes();
    }

    // ---- reading -----------------------------------------------------------

    /// Flash is memory-mapped: a read is a load, and a read of an erased
    /// cell returns the erased pattern. Byte-wise on purpose - the
    /// caller's buffer has no alignment obligation and this core cannot
    /// do an unaligned word load at all.
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        const volatile uint8_t* src = reinterpret_cast<const volatile uint8_t*>(addr);
        for (uint32_t i = 0; i < dst.size(); ++i) {
            dst[i] = src[i];
        }
    }

    /// The OTP area (3.3.1): 1 Kbyte of double words, writable ONCE and
    /// never erasable - not even back to zero. There is no write verb
    /// here on purpose; this is the read.
    static bool read_otp(uint32_t offset, std::span<uint8_t> dst) {
        if (offset > otp_size || dst.size() > otp_size - offset) {
            return false;
        }
        read(otp_base + offset, dst);
        return true;
    }

    // ---- status ------------------------------------------------------------

    static uint32_t status() { return FLASH->SR; }
    static bool busy() { return (FLASH->SR & FlashFlag::busy) != 0u; }
    static bool config_busy() { return (FLASH->SR & FlashFlag::config_busy) != 0u; }
    static uint32_t errors() { return FLASH->SR & FlashFlag::errors; }

    /// Write-one-to-clear, and only the bits that are.
    static void clear(uint32_t mask) { FLASH->SR = mask & FlashFlag::clearable; }
    static void clear_errors() { clear(FlashFlag::clearable); }

    /// The whole FLASH_SR the last operation ended on, EOP and the busy
    /// bits included - the flags this file's own return value drops.
    /// Diagnostic; a suite reads it, no decision here consults it.
    static uint32_t last_status() { return last_sr_; }

    /**
     * Wait until the engine is idle: neither bank busy AND CFGBSY down.
     * Bounded; false means it never came back, and every caller turns
     * that into FlashFlag::refused rather than proceeding.
     *
     * BOTH conditions matter. BSY1/BSY2 say a bank is working; CFGBSY
     * says FLASH_CR is untouchable, and 3.7.4 warns it is also set by
     * the first byte of an access to a LOCKED flash - so a wait on BSY
     * alone can return while a store to FLASH_CR still hard-faults.
     */
    static bool wait_ready() {
        for (uint32_t spins = 0; spins < wait_spins; ++spins) {
            if ((FLASH->SR & (FlashFlag::busy | FlashFlag::config_busy)) == 0u) {
                turns_ = spins;
                return true;
            }
        }
        turns_ = wait_spins;
        return false;
    }

    /**
     * How many turns the last OPERATION's completing wait spent.
     * Diagnostic, like last_status() - and the one number that measures
     * read-while-write (3.3.9) rather than assuming it: it is the count
     * of loop iterations the CPU completed, out of the bank it is
     * executing from, WHILE the flash was working. A CPU whose
     * instruction fetches were stalled cannot complete any.
     *
     * Only erase_page(), mass_erase(), program(), fast_program_row() and
     * provoke() publish it. The bookkeeping waits inside unlock(),
     * lock() and interrupts() do NOT - they return on their first turn
     * and would otherwise wipe the measurement the caller just made,
     * which is exactly what the first version of this did.
     */
    static uint32_t last_wait_turns() { return op_turns_; }

    // ---- the lock ----------------------------------------------------------

    static bool locked() { return (FLASH->CR & FLASH_CR_LOCK) != 0u; }
    static bool option_locked() { return (FLASH->CR & FLASH_CR_OPTLOCK) != 0u; }

    /**
     * KEY1 then KEY2 (3.3.6). Idempotent: an already-unlocked FLASH_CR
     * is left alone, because writing a key into an unlocked KEYR is
     * itself the "wrong sequence" that locks the register until the next
     * system reset AND raises a bus error - i.e. a HardFault. That is
     * why there is no unconditional unlock verb in this file and why
     * nothing here writes KEYR twice in any other order.
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

    /// Set LOCK again. Bounded like everything else that writes CR.
    static bool lock() {
        if (!wait_ready()) {
            return false;
        }
        FLASH->CR = FLASH->CR | FLASH_CR_LOCK;
        return locked();
    }

    // ---- erase -------------------------------------------------------------

    /**
     * Erase the 2 Kbyte page containing `addr` (3.3.7). `addr` must BE
     * the page's first byte: rounding it here would let a caller erase a
     * neighbour by a typo.
     *
     * ON A DUAL-BANK PART THIS DOES NOT STALL A READ OF THE OTHER BANK
     * (3.3.9), which is the property stm32g0/nvm_flash.hpp is built on.
     * Erasing the bank the CODE runs from stalls every instruction fetch
     * until it finishes (3.3.6) - legal only from RAM, and nothing in
     * brio does it.
     */
    static uint32_t erase_page(uint32_t addr) {
        if (!in_main_flash(addr) || (addr % page_size) != 0u) {
            return FlashFlag::refused;
        }
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();

        const uint32_t bank_bit =
            erase_bank_of(addr) == FlashBank::bank2 ? flash_cr_bank_select : 0u;
        const uint32_t cr = (FLASH->CR & keep_mask) | FLASH_CR_PER | bank_bit |
                            (page_of(addr) << FLASH_CR_PNB_Pos);
        FLASH->CR = cr;
        FLASH->CR = cr | FLASH_CR_STRT;
        const bool done = wait_ready();
        op_turns_ = turns_;
        last_sr_ = FLASH->SR;
        const bool released = release_cr();
        clear(last_sr_);
        return done && released ? (last_sr_ & FlashFlag::errors)
                                : FlashFlag::refused;
    }

    /**
     * Erase a whole bank (3.3.7's "mass erase", which on a dual-bank part
     * is per bank: MER1 and MER2 are separate bits).
     *
     * EXPOSED AND NEVER CALLED IN THIS TREE. On this board bank 1 holds
     * the running image, so a mass erase of it is a way to end a
     * session; bank 2 is the storage attic and erasing it wholesale is
     * slower than erasing the pages that are actually used. It is here
     * because the chapter has it and a bootloader would want it.
     */
    static uint32_t mass_erase(FlashBank bank) {
        const uint32_t bit = bank == FlashBank::bank2 ? flash_cr_mass_erase2
                                                      : FLASH_CR_MER1;
        if (bit == 0u || (bank == FlashBank::bank2 && bank_count() < 2)) {
            return FlashFlag::refused;
        }
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();
        const uint32_t cr = (FLASH->CR & keep_mask) | bit;
        FLASH->CR = cr;
        FLASH->CR = cr | FLASH_CR_STRT;
        const bool done = wait_ready();
        op_turns_ = turns_;
        last_sr_ = FLASH->SR;
        const bool released = release_cr();
        clear(last_sr_);
        return done && released ? (last_sr_ & FlashFlag::errors)
                                : FlashFlag::refused;
    }

    // ---- program -----------------------------------------------------------

    /**
     * Standard programming (3.3.8): whole double words, one high-voltage
     * pulse each.
     *
     * `addr` and `src.size()` are multiples of cell_size and the target
     * cells have been erased since they were last written - the same
     * contract util/nv_heap.hpp's FlashMedia states, and the same one
     * the silicon states in the other direction (PGAERR for the
     * alignment, PROGERR for the second write). Both halves are checked
     * HERE before the flash is touched, so a caller's arithmetic slip
     * costs a false and not an error flag storm.
     *
     * The two stores per cell are 32-bit and volatile: a byte or
     * half-word store into the flash raises SIZERR (3.3.8), and the
     * width of a store is exactly the kind of thing an optimizer is
     * otherwise free to change.
     */
    static uint32_t program(uint32_t addr, std::span<const uint8_t> src) {
        if (src.empty() || (addr % cell_size) != 0u ||
            (src.size() % cell_size) != 0u) {
            return FlashFlag::refused;
        }
        if (!in_main_flash(addr) ||
            src.size() > base + size_bytes() - addr) {
            return FlashFlag::refused;
        }
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();

        FLASH->CR = (FLASH->CR & keep_mask) | FLASH_CR_PG;
        uint32_t err = 0;
        for (uint32_t off = 0; off < src.size(); off += cell_size) {
            volatile uint32_t* at =
                reinterpret_cast<volatile uint32_t*>(addr + off);
            at[0] = load_le32(src.data() + off);
            at[1] = load_le32(src.data() + off + 4);
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
        if (!release_cr()) {
            err |= FlashFlag::refused;
        }
        return err;
    }

    /**
     * Fast programming (3.3.8): one whole ROW of 32 double words under a
     * single high-voltage ramp, which is what makes it faster than 32
     * ordinary programs.
     *
     * THREE OBLIGATIONS THE SILICON PUTS ON THE CALLER, all stated
     * because none of them can be enforced from in here:
     *  - the row must have been erased since it was last written (3.3.8
     *    step 1, else PGSERR);
     *  - HCLK must be at least 8 MHz, and the 32 double words must be
     *    written SUCCESSIVELY - at most about 20 us apart, or the
     *    silicon raises MISSERR and stops. So the caller masks
     *    interrupts around this call. It is not done here because a
     *    driver that masks interrupts behind the caller's back is a
     *    driver that decides the system's latency, and this stratum's
     *    standing rule is that pacing belongs to the owner;
     *  - the whole row must be programmed within 7 ms of setting FSTPG,
     *    counted by the silicon's own time-out, which raises FASTERR.
     *
     * The loop below therefore does NOT wait between double words. It is
     * the one place in this file that stores into the flash without
     * checking the status in between - by contract, not by omission.
     */
    static uint32_t fast_program_row(uint32_t addr, std::span<const uint8_t> src) {
        if (src.size() != row_size || (addr % row_size) != 0u) {
            return FlashFlag::refused;
        }
        if (!in_main_flash(addr) || row_size > base + size_bytes() - addr) {
            return FlashFlag::refused;
        }
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();

        FLASH->CR = (FLASH->CR & keep_mask) | FLASH_CR_FSTPG;
        volatile uint32_t* at = reinterpret_cast<volatile uint32_t*>(addr);
        for (uint32_t i = 0; i < cells_per_row * 2u; i += 2u) {
            at[i] = load_le32(src.data() + i * 4u);
            at[i + 1u] = load_le32(src.data() + i * 4u + 4u);
        }
        const bool done = wait_ready();
        op_turns_ = turns_;
        last_sr_ = FLASH->SR;
        const bool released = release_cr();
        clear(last_sr_);
        return done && released ? (last_sr_ & FlashFlag::errors)
                                : FlashFlag::refused;
    }

    // ---- the three malformed sequences, on purpose -------------------------

    /**
     * 3.3.8 describes three ways to get a program wrong, and each has a
     * flag of its own. Everything else in this file REFUSES those three
     * before the flash is touched - which is right for an application
     * and useless for a bench suite, whose job is to see the SILICON
     * raise SIZERR, PGAERR and PGSERR rather than to see this file's
     * bounds check.
     *
     * So they are spelled here, once, deliberately named, and nothing
     * but a suite has any business calling them. `addr` must be a
     * double-word-aligned address in a page the caller owns and does not
     * mind losing: a misstep aborts its own operation, so nothing is
     * programmed, but the cell is left in whatever state the abort chose.
     *
     * WHAT DECIDES WHETHER A MISSTEP IS RECOVERABLE IS NOT ITS FLAG BUT
     * ITS LENGTH, which no chapter says and the bench measured. 3.7.4
     * makes CFGBSY fall only when "a complete double word sent to the
     * flash memory" is finished, so:
     *
     *  - the first three below each send eight bytes' worth of accesses,
     *    in the wrong shape. Each raises its documented flag and the
     *    engine comes back;
     *  - `half_double_word` sends a perfectly legal FIRST word and no
     *    second. No flag rises at all, CFGBSY stands until the next
     *    SYSTEM RESET, no further flash operation is possible and a
     *    store into FLASH_CR would be a HardFault (3.7.5). provoke()
     *    returns FlashFlag::refused and touches nothing.
     *
     * A suite that stages these therefore stages them the way it stages
     * a watchdog: one per boot, with a reset behind it.
     */
    enum class Misstep : uint8_t {
        half_word_store,        ///< SIZERR: only word accesses may program
        unaligned_double_word,  ///< PGAERR: the first word must be 8-aligned
        store_without_pg,       ///< PGSERR: a write with PG and FSTPG clear
        half_double_word,       ///< no flag at all: the engine simply waits
    };

    static uint32_t provoke(Misstep step, uint32_t addr) {
        if (!in_main_flash(addr) || (addr % cell_size) != 0u) {
            return FlashFlag::refused;
        }
        if (locked() || !wait_ready()) {
            return FlashFlag::refused;
        }
        clear_errors();

        if (step != Misstep::store_without_pg) {
            FLASH->CR = (FLASH->CR & keep_mask) | FLASH_CR_PG;
        }
        // EACH MISSTEP SENDS A WHOLE DOUBLE WORD'S WORTH OF DATA, in the
        // wrong shape. That is not politeness: 3.7.4 makes CFGBSY fall
        // only when "a complete double word sent to the flash memory" is
        // finished, so a misstep that stops halfway cannot report
        // anything at all - the engine is still waiting for the rest.
        switch (step) {
        case Misstep::half_word_store: {
            // Eight bytes, in four HALF-WORD accesses. Only word
            // accesses may program (3.3.8), so every one of them is the
            // offence.
            volatile uint16_t* const at =
                reinterpret_cast<volatile uint16_t*>(addr);
            at[0] = 0x1234u;
            at[1] = 0x5678u;
            at[2] = 0x9ABCu;
            at[3] = 0xDEF0u;
            break;
        }
        case Misstep::unaligned_double_word: {
            // Two words, both 32-bit, but straddling the boundary: the
            // first at +4 is not a double word's own address and the
            // second at +8 does not belong to the same double word as
            // the first - 3.3.8 names both conditions for PGAERR.
            volatile uint32_t* const at =
                reinterpret_cast<volatile uint32_t*>(addr);
            at[1] = 0x1234'5678UL;
            at[2] = 0x9ABC'DEF0UL;
            break;
        }
        case Misstep::store_without_pg:
            reinterpret_cast<volatile uint32_t*>(addr)[0] = 0x1234'5678UL;
            reinterpret_cast<volatile uint32_t*>(addr)[1] = 0x9ABC'DEF0UL;
            break;
        case Misstep::half_double_word:
            // The shape that is not an error at all and is worse than
            // one: a perfectly legal FIRST word with no second. The
            // engine is still waiting, CFGBSY never falls, and only a
            // reset gets the flash interface back.
            reinterpret_cast<volatile uint32_t*>(addr)[0] = 0x1234'5678UL;
            break;
        }
        const bool done = wait_ready();
        op_turns_ = turns_;
        last_sr_ = FLASH->SR;
        if (config_busy()) {
            // MEASURED, AND IT IS WHY THIS VERB CARRIES A WARNING: a
            // misstep that sent the flash HALF a double word leaves
            // CFGBSY standing for good. 3.7.4 says the flag falls only
            // when "a complete double word sent to the flash memory" is
            // finished, and on this silicon completing the pair
            // afterwards does NOT do it - so FLASH_CR stays untouchable
            // (a store there is a HardFault), PG stays set, and the only
            // way out is a reset. Nothing is written here and the caller
            // is told; last_status() still holds the flag the misstep
            // raised.
            return FlashFlag::refused;
        }
        if (!release_cr()) {
            return FlashFlag::refused;
        }
        clear(last_sr_);
        return done ? (last_sr_ & FlashFlag::errors) : FlashFlag::refused;
    }

    // ---- the interrupt -----------------------------------------------------

    static constexpr IRQn_Type irq() { return FLASH_IRQn; }

    /**
     * The three interrupt enables of FLASH_CR (table 24). Note what EOP
     * and OPERR really are: 3.7.4 says each is SET ONLY IF its enable
     * bit is set, so these are not merely interrupt masks - they decide
     * whether the flag exists at all. That is why this file never waits
     * on EOP.
     */
    static bool interrupts(bool end_of_operation, bool error, bool read_protect) {
        if (!wait_ready()) {
            return false;
        }
        uint32_t cr = FLASH->CR &
                      ~(FLASH_CR_EOPIE | FLASH_CR_ERRIE | FLASH_CR_RDERRIE);
        if (end_of_operation) cr |= FLASH_CR_EOPIE;
        if (error) cr |= FLASH_CR_ERRIE;
        if (read_protect) cr |= FLASH_CR_RDERRIE;
        FLASH->CR = cr;
        return true;
    }

    /**
     * The FLASH_IRQHandler body an app binds to the vector. Returns the
     * flags it found and clears exactly those, so a handler can report
     * without a second read racing the next operation.
     *
     * The ECC DOUBLE error is NOT here: 3.3.3 raises an NMI for it, not
     * this line, so it is the NMI handler's to read through ecc().
     */
    [[gnu::always_inline]] static inline uint32_t isr() {
        const uint32_t sr = FLASH->SR & FlashFlag::clearable;
        if (sr != 0u) {
            FLASH->SR = sr;
        }
        uint32_t ecc_seen = 0;
        if ((FLASH->ECCR & FLASH_ECCR_ECCC) != 0u) {
            FLASH->ECCR = FLASH_ECCR_ECCC;
            ecc_seen |= 1u;
        }
        volatile uint32_t* const r2 = flash_ecc2r();
        if (r2 != nullptr && (*r2 & FLASH_ECCR_ECCC) != 0u) {
            *r2 = FLASH_ECCR_ECCC;
            ecc_seen |= 2u;
        }
        last_isr_ecc_ = ecc_seen;
        return sr;
    }

    /// Which ECC registers the last isr() found a correction in: bit 0
    /// bank 1, bit 1 bank 2.
    static uint32_t last_isr_ecc() { return last_isr_ecc_; }

    // ---- ECC ---------------------------------------------------------------

    /**
     * FLASH_ECCR / FLASH_ECCR2 (3.3.3). One wrong bit in a 72-bit word
     * is corrected and reported in ECCC; two are detected and raise an
     * NMI with ECCD set. ADDR_ECC is a DOUBLE-WORD offset, its low three
     * bits always zero.
     *
     * The chapter's own two caveats, worth knowing before believing a
     * reading: while either flag stands the register is FROZEN (a later
     * error does not update it), and a re-read of the failing address
     * may report nothing at all if the value is still in the cache - so
     * a diagnosis that wants to re-provoke the error resets the cache
     * first (FlashAccel::flush_instruction_cache()).
     */
    static FlashEccStatus ecc(FlashBank bank) {
        FlashEccStatus s{};
        const volatile uint32_t* reg =
            bank == FlashBank::bank2 ? flash_ecc2r() : &FLASH->ECCR;
        if (reg == nullptr) {
            return s;
        }
        s.present = true;
        s.raw = *reg;
        s.corrected = (s.raw & FLASH_ECCR_ECCC) != 0u;
        s.detected = (s.raw & FLASH_ECCR_ECCD) != 0u;
        s.system_flash = (s.raw & FLASH_ECCR_SYSF_ECC) != 0u;
        s.double_word_offset = s.raw & FLASH_ECCR_ADDR_ECC_Msk;
        return s;
    }

    static void clear_ecc(FlashBank bank) {
        volatile uint32_t* reg =
            bank == FlashBank::bank2 ? flash_ecc2r() : &FLASH->ECCR;
        if (reg != nullptr) {
            *reg = FLASH_ECCR_ECCC | FLASH_ECCR_ECCD;
        }
    }

    /// ECCCIE, per bank. The DOUBLE-error NMI has no enable at all.
    static void ecc_correction_interrupt(FlashBank bank, bool on) {
        volatile uint32_t* reg =
            bank == FlashBank::bank2 ? flash_ecc2r() : &FLASH->ECCR;
        if (reg == nullptr) {
            return;
        }
        // ECCC/ECCD are write-one-to-clear in the same register, so the
        // read-modify-write must not carry them back down.
        const uint32_t kept = *reg & ~(FLASH_ECCR_ECCC | FLASH_ECCR_ECCD |
                                       FLASH_ECCR_ECCCIE);
        *reg = kept | (on ? FLASH_ECCR_ECCCIE : 0u);
    }

private:
    /**
     * Put FLASH_CR back where the next operation is legal - but only if
     * that store is legal at all.
     *
     * 3.7.5: a write to FLASH_CR while CFGBSY stands "causes a HardFault
     * exception". Every operation here ends by clearing its own PG / PER
     * / MER bit, and a wait that timed out is exactly the case where
     * doing so would kill the board instead of reporting. So the store
     * is conditional and false says the register was left alone - which
     * every caller turns into FlashFlag::refused.
     */
    static bool release_cr() {
        if (config_busy()) {
            return false;
        }
        FLASH->CR = FLASH->CR & keep_mask;
        return true;
    }

    /// The bits of FLASH_CR every operation must leave alone, and the
    /// operation bits every one of them must clear first: 3.3.8 makes
    /// PGSERR the punishment for setting PG while PER or MER is up, and
    /// vice versa, so each verb starts from a CR with all five down.
    static constexpr uint32_t keep_mask =
        ~(FLASH_CR_PG | FLASH_CR_PER | FLASH_CR_MER1 | flash_cr_mass_erase2 |
          FLASH_CR_FSTPG | FLASH_CR_PNB | flash_cr_bank_select);

    /// Little-endian load from a possibly unaligned byte buffer. This
    /// core traps on an unaligned 32-bit load, and gcc emits four byte
    /// loads for exactly this shape.
    static uint32_t load_le32(const uint8_t* at) {
        return static_cast<uint32_t>(at[0]) |
               (static_cast<uint32_t>(at[1]) << 8) |
               (static_cast<uint32_t>(at[2]) << 16) |
               (static_cast<uint32_t>(at[3]) << 24);
    }

    static inline uint32_t last_sr_ = 0;
    /// Every wait's own count; only an operation copies it out.
    static inline uint32_t turns_ = 0;
    static inline uint32_t op_turns_ = 0;
    static inline uint32_t last_isr_ecc_ = 0;
};

// ---- the option bytes, read-only --------------------------------------------

/// RDP as 3.7.8 codes it: 0xAA is level 0, 0xCC is level 2, ANYTHING
/// ELSE is level 1. There is no fourth possibility, which is why the
/// decode is a function and not a table.
enum class FlashRdpLevel : uint8_t { level0 = 0, level1 = 1, level2 = 2 };

/// One WRP area, in PAGE offsets within its bank (3.7.11). An area is
/// EMPTY when start > end, which is how the erased option value
/// (start 0x7F, end 0x00 after loading) says "nothing is protected".
struct FlashWrpArea {
    uint8_t start = 0;
    uint8_t end = 0;
    bool empty() const { return start > end; }
};

/// One PCROP area, in SUBPAGE offsets (512 bytes, 3.7.9). Same emptiness
/// rule.
struct FlashPcropArea {
    uint16_t start = 0;
    uint16_t end = 0;
    bool empty() const { return start > end; }
};

/**
 * FLASH_OPTR and the protection registers, DECODED AND NEVER WRITTEN.
 *
 * Everything here is a plain read of an option register the option
 * loader filled at power-on reset. There is no setter anywhere, and that
 * is the design: RDP Level 2 is one-way, ES0548 2.2.9 turns an
 * interrupted option write into a device with no debug port, and the
 * other two targets already put this class of change in a TOOL over the
 * programming interface rather than in firmware (tools/bench.py's
 * `fuses` verb). What firmware needs is to KNOW - to cross-check the
 * watchdog options against reset.hpp's registers, to see the BOR level,
 * to prove the bank mapping the storage backend assumes.
 */
struct FlashOptions {
    FlashOptions() = delete;

    static uint32_t raw() { return FLASH->OPTR; }

    static uint8_t rdp_code() {
        return static_cast<uint8_t>(FLASH->OPTR & FLASH_OPTR_RDP_Msk);
    }
    static FlashRdpLevel rdp() {
        const uint8_t c = rdp_code();
        if (c == 0xAAu) return FlashRdpLevel::level0;
        if (c == 0xCCu) return FlashRdpLevel::level2;
        return FlashRdpLevel::level1;
    }

    static bool bor_enabled() { return (FLASH->OPTR & FLASH_OPTR_BOR_EN) != 0u; }
    /// BORR_LEV / BORF_LEV, 0..3. 3.7.8 gives the thresholds: rising
    /// 2.1/2.3/2.6/2.9 V, falling 2.0/2.2/2.5/2.8 V.
    static uint8_t bor_rising_level() {
        return static_cast<uint8_t>((FLASH->OPTR & FLASH_OPTR_BORR_LEV_Msk) >>
                                    FLASH_OPTR_BORR_LEV_Pos);
    }
    static uint8_t bor_falling_level() {
        return static_cast<uint8_t>((FLASH->OPTR & FLASH_OPTR_BORF_LEV_Msk) >>
                                    FLASH_OPTR_BORF_LEV_Pos);
    }

    // The three nRST_* bits are spelled here the way they READ, not the
    // way they are named: the option bit is 1 for "no reset", so a verb
    // called reset_on_stop() has to be its complement or the name lies.
    static bool reset_on_stop() { return (FLASH->OPTR & FLASH_OPTR_nRST_STOP) == 0u; }
    static bool reset_on_standby() { return (FLASH->OPTR & FLASH_OPTR_nRST_STDBY) == 0u; }
    static bool reset_on_shutdown() { return (FLASH->OPTR & FLASH_OPTR_nRST_SHDW) == 0u; }

    /// IWDG_SW = 1 means the SOFTWARE watchdog: it is started by
    /// firmware. 0 is the hardware one, which runs from the boot.
    static bool iwdg_software() { return (FLASH->OPTR & FLASH_OPTR_IWDG_SW) != 0u; }
    static bool iwdg_runs_in_stop() { return (FLASH->OPTR & FLASH_OPTR_IWDG_STOP) != 0u; }
    static bool iwdg_runs_in_standby() { return (FLASH->OPTR & FLASH_OPTR_IWDG_STDBY) != 0u; }
    static bool wwdg_software() { return (FLASH->OPTR & FLASH_OPTR_WWDG_SW) != 0u; }

    /// RAM_PARITY_CHECK is 1 for DISABLED (3.7.8), so this reads the
    /// meaning rather than the bit.
    static bool ram_parity_check() {
        return (FLASH->OPTR & FLASH_OPTR_RAM_PARITY_CHECK) == 0u;
    }

    /// The bank configuration as the option bytes hold it. Flash::
    /// bank_count() is the one to ask what the part actually does - on a
    /// 512 KB device this bit is "only effective for 256-Kbyte devices"
    /// (3.7.8) and says nothing.
    static bool dual_bank_bit() {
        return flash_optr_dual_bank != 0u &&
               (FLASH->OPTR & flash_optr_dual_bank) != 0u;
    }
    static bool banks_swapped() { return Flash::banks_swapped(); }

    static bool boot0_from_option() {
        return (FLASH->OPTR & FLASH_OPTR_nBOOT_SEL) != 0u;
    }
    static bool nboot0() { return (FLASH->OPTR & FLASH_OPTR_nBOOT0) != 0u; }
    static bool nboot1() { return (FLASH->OPTR & FLASH_OPTR_nBOOT1) != 0u; }

    /// NRST_MODE, 3.7.8: 1 = reset input only, 2 = GPIO, 3 =
    /// bidirectional (the legacy default). 0 is Reserved.
    static uint8_t nrst_mode() {
        return static_cast<uint8_t>((FLASH->OPTR & FLASH_OPTR_NRST_MODE_Msk) >>
                                    FLASH_OPTR_NRST_MODE_Pos);
    }
    static bool internal_reset_holder() {
        return (FLASH->OPTR & FLASH_OPTR_IRHEN) != 0u;
    }

    // ---- the protection areas ----------------------------------------------

    /// WRP area A or B of a bank, in page offsets. Area index 0 = A,
    /// 1 = B. An absent bank answers an empty area.
    static FlashWrpArea wrp(FlashBank bank, uint8_t area) {
        const volatile uint32_t* reg = nullptr;
        if (bank == FlashBank::bank1) {
            reg = area == 0 ? &FLASH->WRP1AR : &FLASH->WRP1BR;
        } else {
            reg = area == 0 ? flash_wrp2ar() : flash_wrp2br();
        }
        if (reg == nullptr) {
            return FlashWrpArea{0xFFu, 0u};
        }
        const uint32_t v = *reg;
        return FlashWrpArea{static_cast<uint8_t>(v & 0x7Fu),
                            static_cast<uint8_t>((v >> 16) & 0x7Fu)};
    }

    /// PCROP area A or B of a bank, in 512-byte subpage offsets.
    static FlashPcropArea pcrop(FlashBank bank, uint8_t area) {
        const volatile uint32_t* start = nullptr;
        const volatile uint32_t* end = nullptr;
        if (bank == FlashBank::bank1) {
            start = area == 0 ? &FLASH->PCROP1ASR : &FLASH->PCROP1BSR;
            end = area == 0 ? &FLASH->PCROP1AER : &FLASH->PCROP1BER;
        } else {
            start = area == 0 ? flash_pcrop2a_start() : flash_pcrop2b_start();
            end = area == 0 ? flash_pcrop2a_end() : flash_pcrop2b_end();
        }
        if (start == nullptr || end == nullptr) {
            return FlashPcropArea{0x1FFu, 0u};
        }
        return FlashPcropArea{static_cast<uint16_t>(*start & 0x1FFu),
                              static_cast<uint16_t>(*end & 0x1FFu)};
    }

    /// PCROP_RDP (3.7.10): whether the PCROP area is erased when the RDP
    /// level goes back to 0.
    static bool pcrop_erased_on_rdp_regression() {
        return (FLASH->PCROP1AER & FLASH_PCROP1AER_PCROP_RDP) != 0u;
    }

    /// The securable memory area, in pages from the bottom of the bank
    /// (3.7.21). Zero = none defined, which is what makes FLASH_CR's
    /// SEC_PROT bits inert.
    static uint8_t securable_pages(FlashBank bank) {
        if (!flash_securable_capable) {
            return 0;
        }
        const uint32_t v = FLASH->SECR;
        if (bank == FlashBank::bank2) {
            return flash_secr_sec_size2 == 0u
                       ? 0u
                       : static_cast<uint8_t>((v & flash_secr_sec_size2) >> 20);
        }
        return static_cast<uint8_t>(v & FLASH_SECR_SEC_SIZE_Msk);
    }

    /// BOOT_LOCK: the boot is forced from the user area. Set together
    /// with RDP level 1 it is what ES0548 2.2.9 warns about - the state a
    /// mismatched option write can leave a board in, with no way back.
    static bool boot_lock() { return (FLASH->SECR & FLASH_SECR_BOOT_LOCK) != 0u; }
};

} // namespace brio
