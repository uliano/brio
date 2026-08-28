/*
 * nvm.hpp
 *
 * The SAM C21 Non-Volatile Memory Controller (DS60001479M ch. 27): the
 * one resource through which flash is erased, programmed, protected and
 * described.
 *
 *  Nvm            the BLOCK - geometry (from the device header, checked
 *                 against PARAM), read wait states and the cache, the
 *                 command interface with its CMDEX key and error
 *                 reporting, erase/program on EITHER array, the region
 *                 lock verbs, and the read-only factory views (user row,
 *                 calibration areas, the 128-bit serial number).
 *
 *  FlashWaitStates  the wait-state verb the clock code needs. It lives
 *                 here because it is NVMCTRL's register (CTRLB.RWS);
 *                 samc/clock.hpp calls it, having held it on loan until
 *                 this driver existed.
 *
 * WHAT THE SILICON DOES, in the four facts that shape every verb below.
 *
 * 1. TWO PROGRAMMABLE ARRAYS, ONE PAGE BUFFER. The main array (256 KB at
 *    0x00000000 on the 18A parts) and a dedicated RWWEE array (8 KB at
 *    0x00400000) share the page buffer, the ADDR register and the command
 *    register - what differs is the command code and, decisively, WHO
 *    STALLS: reading the main array while the main array is programmed or
 *    erased stalls the AHB until the operation ends, while the main array
 *    can be read freely while the RWWEE array is written (27.6.4.1 and
 *    27.6.4.2). Code executes from the main array, so an RWWEE write is
 *    the only one a running program does not pay for in frozen cycles.
 *    Measured, and the difference is not subtle: through a row erase the
 *    CPU completes about 3950 polling turns on RWWEE and ONE on the main
 *    array. The RWWEE array is in exchange NOT CACHED, so reads from it
 *    are slower and full-word accesses take twice the data phase - and it
 *    is the more DURABLE of the two, 100k cycles against the main array's
 *    25k (tables 45-43 and 45-44). Both halves of that trade point the
 *    same way for stored records, which is why samc/nvm_flash.hpp puts
 *    the heap there.
 *
 * 2. ERASE BY ROW, PROGRAM BY PAGE. A page is 64 bytes, a row is four
 *    pages = 256 bytes; erase granularity is the row, program granularity
 *    the page (27.6.2). This is exactly util/nv_heap.hpp's `erase_size` /
 *    `write_cell` split, and samc/nvm_flash.hpp is where the two meet.
 *
 * 3. THE PAGE BUFFER IS LOADED BY WRITING TO THE MEMORY-MAPPED ARRAY, and
 *    it has two traps. Writes must be 16 or 32 bits wide - AN 8-BIT WRITE
 *    RAISES A SYSTEM EXCEPTION (27.6.4.3), which on this core is a
 *    HardFault, so every store this header makes is a 32-bit one. And the
 *    64-bit holding register PBLDATA is reset to all ones whenever a write
 *    CROSSES a 64-bit boundary, which means a random-access write pattern
 *    silently fills the other half of each 64-bit section with ones
 *    (27.6.4.3's own second example).
 *
 *    THE REAL RULE IS NARROWER THAN "WRITE ASCENDING", and it was
 *    measured rather than inferred (test_samc_nvm letter c): what a
 *    section needs is that ITS TWO WORDS BE WRITTEN BACK TO BACK.
 *    Ascending satisfies that; so does DESCENDING, which crosses a
 *    boundary on every store and still comes out byte-exact, because
 *    15-then-14 completes a section before the crossing. What actually
 *    loses data is interleaving - all even words then all odd words
 *    leaves every even word erased, which is the chapter's own example
 *    generalized. program_page() writes ascending, which is the simplest
 *    way to satisfy the rule.
 *
 *    A SEPARATE LIMIT, and it is not in chapter 27 at all but in table
 *    45-42's footnote: AT MOST 8 CONSECUTIVE WRITES ARE ALLOWED PER ROW
 *    before a row erase becomes mandatory. A row is four pages, so
 *    programming every page of a row once is 4 and safe; a caller that
 *    rewrites pages of a live row without erasing must count.
 *
 * 4. ADDR IS A HALF-WORD OFFSET FROM THE SECTION BASE, not a byte address
 *    and not an absolute one: "the effective address for the operation is
 *    Start address of the section + 2*ADDR" (27.8.8). MEASURED on the
 *    bench chip rather than trusted, because 22 bits could have held
 *    either convention: a page-buffer load at 0x00400100 (RWWEE) leaves
 *    ADDR = 0x00000080 and one at 0x00020100 (main) leaves 0x00010080 -
 *    section-relative in both cases, as the chapter says.
 *
 * ERRATA: NVMCTRL HAS NONE. Item 1.14.1 of DS80000740S reads "Reserved"
 * with dashes across every silicon revision; the revision history records
 * that the one item this module ever had ("EEPROM Cache") was deprecated
 * due to resolution in the rev. K document (2021). Code written against
 * older SAMD/SAMC examples may still carry a cache workaround around
 * RWWEE access - it is not needed here.
 *
 * REGISTER ACCESS PROTECTION: CTRLA, CTRLB and ADDR are PAC
 * write-protected (27.5.5). PAC protection is off out of reset and no
 * brio driver enables it, so nothing here performs the unlock dance; when
 * a PAC driver arrives it must, and this is the note that says so.
 *
 * NOT BUILT (docs/samc/nvm.md carries the list and the reasons):
 *  - The SSB command (Set Security Bit). It is ONE-WAY - only a debugger
 *    chip erase clears it - and its whole effect is to lock the part
 *    against the debugger. `security_bit()` reads the state; nothing here
 *    sets it, the same ruling that kept CHER and software fuse writes out
 *    of avrdx/nvm.hpp.
 *  - Writing the NVM User Row (the EAR and WAP commands). On this family
 *    the user row IS the fuses - BOOTPROT, EEPROM size, BODVDD level and
 *    action, and the watchdog's power-on ENABLE/ALWAYSON/PER - and it
 *    survives a chip erase, so a wrong word is not recoverable by
 *    reflashing. It is read here and typed; writing it is provisioning,
 *    which on the AVR side lives in the bench tool and over UPDI, and
 *    which here wants a bench.py verb over SWD.
 *  - The two commands the device header carries but chapter 27's command
 *    table does not list at all: SF (0xA, "Security Flow") and WL (0xF,
 *    "Write lockbits"). Undocumented and, in WL's case, permanent.
 *  - Power reduction (SPRM/CPRM and CTRLB.SLEEPPRM) is exposed as
 *    configuration and verbs, but the sleep modes that give it meaning
 *    belong to the power pass.
 */

#pragma once

#include <stdint.h>

#include <span>

#include "sam.h"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// Which programmable array a command addresses. The two share one page
/// buffer and one ADDR register; the array decides the command code and
/// the section base ADDR counts from.
enum class NvmArray : uint8_t {
    main,   ///< the code array at 0x00000000 - reading it stalls while it is written
    rwwee,  ///< the read-while-write array at 0x00400000 - written without stalling reads of main
};

/// CTRLB.READMODE: how the cache behaves on a miss (27.8.2).
enum class NvmReadMode : uint8_t {
    no_miss_penalty = NVMCTRL_CTRLB_READMODE_NO_MISS_PENALTY_Val,  ///< fastest
    low_power = NVMCTRL_CTRLB_READMODE_LOW_POWER_Val,              ///< a wait state per miss
    deterministic = NVMCTRL_CTRLB_READMODE_DETERMINISTIC_Val,      ///< hit and miss cost the same
};

/// CTRLB.SLEEPPRM: whether the NVM block powers down in sleep, and what
/// wakes it. The reset value is WAKEUPACCESS; brio defaults to `disabled`
/// because nothing on this target sleeps yet and an unexpected wake
/// latency is worse than the microamps until the power pass measures both.
enum class NvmSleepPower : uint8_t {
    wake_on_access = NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS_Val,
    wake_on_exit = NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT_Val,
    disabled = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val,
};

/// What went wrong with a command. `none` is the only success.
enum class NvmError : uint8_t {
    none = 0,
    busy,           ///< INTFLAG.READY was low: a previous command had not finished
    timed_out,      ///< READY never came back within the bounded wait
    bad_address,    ///< the address is not in the array, or not correctly aligned
    program_error,  ///< STATUS.PROGE - invalid command or bad key
    lock_error,     ///< STATUS.LOCKE - the target region is locked
    nvm_error,      ///< STATUS.NVME - the controller reported a programming/erase failure
};

/// STATUS, unpacked (27.8.7). The three error bits are write-one-to-clear
/// and accumulate until cleared, exactly like the AVR's RSTFR.
struct NvmStatus {
    bool nvm_error;       ///< NVME
    bool lock_error;      ///< LOCKE
    bool program_error;   ///< PROGE
    bool page_buffer_loaded;  ///< LOAD
    bool power_reduced;   ///< PRM
    bool security_bit;    ///< SB
};

/// INTFLAG / INTENSET / INTENCLR bits (27.8.4..6).
struct NvmFlag {
    static constexpr uint8_t ready = NVMCTRL_INTFLAG_READY_Msk;
    static constexpr uint8_t error = NVMCTRL_INTFLAG_ERROR_Msk;
};

/**
 * The block's configuration. Wait states are the one field with no safe
 * default: they are a function of the CPU frequency and the clock code
 * computes them, so `init` is normally reached through samc/clock.hpp
 * rather than called by hand.
 */
struct NvmConfig {
    uint8_t wait_states = 0;
    bool cache = true;
    NvmReadMode read_mode = NvmReadMode::no_miss_penalty;
    NvmSleepPower sleep_power = NvmSleepPower::disabled;

    /// CTRLB.MANW. Reset value is 1 (manual), and brio keeps it there:
    /// with automatic writes a store to the LAST word of a page commits
    /// the whole page, which turns a stray write into a flash write, and
    /// a partial page has to be committed by hand anyway (27.6.4.3.2's
    /// own note). The field is here because the chapter has it.
    bool manual_write = true;
};

// =============================================================================
// The block
// =============================================================================

/**
 * NVMCTRL as a monostate resource.
 *
 * GEOMETRY COMES FROM THE DEVICE HEADER, which carries it per variant as
 * instance parameters (NVMCTRL_FLASH_SIZE, NVMCTRL_PAGES, ...): that is
 * the package-variability rule of this framework, and it makes the
 * constants below fold at compile time on every member of the family.
 * PARAM reports the same numbers at run time and `geometry_matches()`
 * compares them - a cheap assertion that the header the code was built
 * against describes the silicon it is running on.
 *
 * NO GENERIC CLOCK. NVMCTRL runs on the AHB and APB clocks alone (the
 * device header's NVMCTRL_GCLK_ID is a test-mode channel, not a
 * functional one), and its APB clock is on out of reset - it must be,
 * since the CPU fetches through this controller. There is nothing to
 * enable before using it.
 */
struct Nvm {
    Nvm() = delete;

    // ---- geometry ---------------------------------------------------------

    /// Bytes in one program unit.
    static constexpr uint32_t page_size = NVMCTRL_PAGE_SIZE;
    /// Bytes in one erase unit.
    static constexpr uint32_t row_size = NVMCTRL_ROW_SIZE;
    static constexpr uint32_t pages_per_row = row_size / page_size;

    static constexpr uint32_t main_base = 0x00000000UL;
    static constexpr uint32_t main_size = NVMCTRL_FLASH_SIZE;
    static constexpr uint32_t main_end = main_base + main_size;
    static constexpr uint32_t main_pages = NVMCTRL_PAGES;

    static constexpr uint32_t rwwee_base = NVMCTRL_RWW_EEPROM_ADDR;
    static constexpr uint32_t rwwee_size = NVMCTRL_RWW_EEPROM_SIZE;
    static constexpr uint32_t rwwee_end = rwwee_base + rwwee_size;
    static constexpr uint32_t rwwee_pages = NVMCTRL_RWWEE_PAGES;

    /// The main array is divided into 16 equally sized lock regions
    /// (27.6.3); the RWWEE array has none - its rows are writable
    /// regardless of lock status.
    static constexpr uint8_t region_count = 16;
    static constexpr uint32_t region_size = main_size / region_count;

    /// The NVM User Row: the fuse row, read at 0x00804000 (9.3). NOT the
    /// device header's NVMCTRL_USER_PAGE_OFFSET, which names the base of
    /// the whole auxiliary space (0x00800000) - a naming trap worth
    /// stating, since the two differ by 0x4000.
    static constexpr uint32_t user_row = 0x00804000UL;
    static constexpr uint32_t aux_base = NVMCTRL_USER_PAGE_OFFSET;

    /// Read-only factory areas (9.4, 9.5, 9.6).
    static constexpr uint32_t software_calibration = 0x00806020UL;
    static constexpr uint32_t temperature_calibration = 0x00806030UL;

    static_assert(page_size == 64u && row_size == 256u,
                  "chapter 27 fixes the page at 64 bytes and the row at four "
                  "pages; a variant that disagrees needs this driver re-read");
    static_assert(main_size == main_pages * page_size, "header geometry disagrees with itself");
    static_assert(rwwee_size == rwwee_pages * page_size, "header geometry disagrees with itself");

    /// Base address of an array, and one past its end.
    static constexpr uint32_t base_of(NvmArray a) {
        return a == NvmArray::main ? main_base : rwwee_base;
    }
    static constexpr uint32_t end_of(NvmArray a) {
        return a == NvmArray::main ? main_end : rwwee_end;
    }
    static constexpr bool in_array(NvmArray a, uint32_t addr) {
        return addr >= base_of(a) && addr < end_of(a);
    }

    /// ADDR's encoding, proven at the bench: the half-word offset from
    /// the base of the addressed section (27.8.8).
    static constexpr uint32_t addr_field(NvmArray a, uint32_t addr) {
        return (addr - base_of(a)) >> 1;
    }

    /// Which lock region a main-array address belongs to.
    static constexpr uint8_t region_of(uint32_t addr) {
        return static_cast<uint8_t>((addr - main_base) / region_size);
    }

    // ---- what PARAM says at run time --------------------------------------

    static uint32_t param() { return NVMCTRL_REGS->NVMCTRL_PARAM; }

    /// PARAM.NVMP: pages in the main array.
    static uint16_t param_main_pages() {
        return static_cast<uint16_t>((param() & NVMCTRL_PARAM_NVMP_Msk) >>
                                     NVMCTRL_PARAM_NVMP_Pos);
    }
    /// PARAM.RWWEEP: pages in the RWWEE array.
    static uint16_t param_rwwee_pages() {
        return static_cast<uint16_t>((param() & NVMCTRL_PARAM_RWWEEP_Msk) >>
                                     NVMCTRL_PARAM_RWWEEP_Pos);
    }
    /// PARAM.PSZ decoded to bytes: the field is an exponent, 8 << PSZ.
    static uint16_t param_page_size() {
        const uint32_t psz = (param() & NVMCTRL_PARAM_PSZ_Msk) >> NVMCTRL_PARAM_PSZ_Pos;
        return static_cast<uint16_t>(8u << psz);
    }

    /// True when the silicon reports the geometry this header was
    /// compiled for. A false here means the wrong device header.
    static bool geometry_matches() {
        return param_page_size() == page_size &&
               param_main_pages() == main_pages &&
               param_rwwee_pages() == rwwee_pages;
    }

    // ---- configuration ----------------------------------------------------

    /**
     * Is this configuration one the silicon actually has?
     *
     * Two ways to be wrong, and both would otherwise be SILENT: RWS is a
     * four-bit field, so a sixteenth wait state masks away to zero and
     * the part runs too fast for its flash; and SLEEPPRM's code 0x2 is
     * Reserved (27.8.2), reachable only by casting an integer into the
     * enum but reachable.
     */
    static constexpr bool config_valid(const NvmConfig& c) {
        return c.wait_states <= (NVMCTRL_CTRLB_RWS_Msk >> NVMCTRL_CTRLB_RWS_Pos) &&
               c.sleep_power != static_cast<NvmSleepPower>(0x2);
    }

    /// Compile-time configuration: an impossible field is a build error.
    template <NvmConfig cfg>
    static void init() {
        static_assert(config_valid(cfg),
                      "NvmConfig: CTRLB.RWS holds 0..15 wait states, and "
                      "SLEEPPRM code 0x2 is Reserved (27.8.2)");
        write_ctrlb(cfg);
    }

    /// Run-time configuration: an impossible field is a false return and
    /// nothing written.
    static bool init(const NvmConfig& cfg) {
        if (!config_valid(cfg)) {
            return false;
        }
        write_ctrlb(cfg);
        return true;
    }

    /// The shared body of the two inits: CTRLB written whole. Nothing in
    /// it is enable-protected and none of it synchronizes - CTRLB is a
    /// plain APB register.
    static void write_ctrlb(const NvmConfig& cfg) {
        NVMCTRL_REGS->NVMCTRL_CTRLB =
            NVMCTRL_CTRLB_RWS(cfg.wait_states) |
            (cfg.manual_write ? NVMCTRL_CTRLB_MANW_Msk : 0u) |
            NVMCTRL_CTRLB_SLEEPPRM(static_cast<uint32_t>(cfg.sleep_power)) |
            (cfg.cache ? 0u : NVMCTRL_CTRLB_CACHEDIS_Msk) |
            NVMCTRL_CTRLB_READMODE(static_cast<uint32_t>(cfg.read_mode));
    }

    static uint32_t ctrlb() { return NVMCTRL_REGS->NVMCTRL_CTRLB; }

    static bool cache_enabled() {
        return (ctrlb() & NVMCTRL_CTRLB_CACHEDIS_Msk) == 0u;
    }
    static void cache(bool on) {
        const uint32_t v = ctrlb() & ~NVMCTRL_CTRLB_CACHEDIS_Msk;
        NVMCTRL_REGS->NVMCTRL_CTRLB = v | (on ? 0u : NVMCTRL_CTRLB_CACHEDIS_Msk);
    }
    static NvmReadMode read_mode() {
        return static_cast<NvmReadMode>((ctrlb() & NVMCTRL_CTRLB_READMODE_Msk) >>
                                        NVMCTRL_CTRLB_READMODE_Pos);
    }
    static bool manual_write() {
        return (ctrlb() & NVMCTRL_CTRLB_MANW_Msk) != 0u;
    }

    // ---- status, flags, interrupts ----------------------------------------

    /// INTFLAG.READY: low from the moment a command is issued until it
    /// completes. "Any commands written while INTFLAG.READY is low will
    /// be ignored" (27.6.4).
    [[gnu::always_inline]] static bool ready() {
        return (NVMCTRL_REGS->NVMCTRL_INTFLAG & NVMCTRL_INTFLAG_READY_Msk) != 0u;
    }

    [[gnu::always_inline]] static uint8_t flags() { return NVMCTRL_REGS->NVMCTRL_INTFLAG; }
    static uint8_t armed() { return NVMCTRL_REGS->NVMCTRL_INTENSET; }
    /// Write-one-to-clear.
    static void clear_flags(uint8_t mask) { NVMCTRL_REGS->NVMCTRL_INTFLAG = mask; }
    /// What a shared handler must act on: raised AND armed.
    [[gnu::always_inline]] static uint8_t pending() {
        return static_cast<uint8_t>(NVMCTRL_REGS->NVMCTRL_INTFLAG &
                                    NVMCTRL_REGS->NVMCTRL_INTENSET);
    }

    static void arm(uint8_t mask) { NVMCTRL_REGS->NVMCTRL_INTENSET = mask; }
    static void disarm(uint8_t mask) { NVMCTRL_REGS->NVMCTRL_INTENCLR = mask; }

    static uint16_t status_bits() { return NVMCTRL_REGS->NVMCTRL_STATUS; }

    static NvmStatus status() {
        const uint16_t s = status_bits();
        return NvmStatus{
            (s & NVMCTRL_STATUS_NVME_Msk) != 0u,
            (s & NVMCTRL_STATUS_LOCKE_Msk) != 0u,
            (s & NVMCTRL_STATUS_PROGE_Msk) != 0u,
            (s & NVMCTRL_STATUS_LOAD_Msk) != 0u,
            (s & NVMCTRL_STATUS_PRM_Msk) != 0u,
            (s & NVMCTRL_STATUS_SB_Msk) != 0u,
        };
    }

    /// The write-one-to-clear half of STATUS: the three error bits plus
    /// LOAD. SB and PRM are read-only state, not history.
    static constexpr uint16_t status_w1c =
        NVMCTRL_STATUS_NVME_Msk | NVMCTRL_STATUS_LOCKE_Msk |
        NVMCTRL_STATUS_PROGE_Msk | NVMCTRL_STATUS_LOAD_Msk;

    static void clear_status(uint16_t mask = status_w1c) {
        NVMCTRL_REGS->NVMCTRL_STATUS = mask;
    }

    /// Read the accumulated error bits and clear them in one step - the
    /// boot-time / post-operation verb, the analog of RSTFR's read-and-
    /// clear on the AVR.
    static NvmStatus take_status() {
        const NvmStatus s = status();
        clear_status();
        return s;
    }

    /// STATUS.SB: set once by the SSB command, cleared only by a debugger
    /// chip erase. This driver reads it and never sets it.
    static bool security_bit() {
        return (status_bits() & NVMCTRL_STATUS_SB_Msk) != 0u;
    }

    /**
     * The ISR body. The app binds NVMCTRL_Handler and calls this; the
     * driver never names a vector.
     *
     * Neither flag needs the command path to be polled: READY says a
     * command finished, ERROR that one of the STATUS error bits went up.
     * Both are cleared here, and STATUS is left standing so the handler's
     * caller can read WHICH error it was.
     */
    [[gnu::always_inline]] static uint8_t isr() {
        const uint8_t p = pending();
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    // ---- the command interface --------------------------------------------

    /// The bounded spin a command is given before it is declared timed
    /// out. Not a timing model - a safety net, so a wedged controller
    /// becomes a false return instead of a hang. A row erase is
    /// milliseconds; this is orders of magnitude above it at any clock
    /// this family reaches. NOTE that a command on the MAIN array stalls
    /// the AHB, so on that array the loop below does not even execute
    /// until the operation is over: the bound only ever does work for
    /// RWWEE commands.
    static constexpr uint32_t command_spin_limit = 4'000'000UL;

    /**
     * Issue one command and wait for it.
     *
     * CMD and the 0xA5 key must reach CTRLA in ONE write ("the key value
     * must be written at the same time as CMD", 27.8.1) - hence the
     * single 16-bit store. A wrong key, or a command issued while a
     * previous one runs, sets STATUS.PROGE rather than failing loudly,
     * so the error bits are cleared BEFORE the command and read after:
     * what this returns is this command's outcome and nobody else's.
     */
    static NvmError command(uint16_t cmd) {
        if (!ready()) {
            return NvmError::busy;
        }
        clear_status(NVMCTRL_STATUS_NVME_Msk | NVMCTRL_STATUS_LOCKE_Msk |
                     NVMCTRL_STATUS_PROGE_Msk);

        NVMCTRL_REGS->NVMCTRL_CTRLA =
            static_cast<uint16_t>(NVMCTRL_CTRLA_CMD(cmd) |
                                  NVMCTRL_CTRLA_CMDEX(NVMCTRL_CTRLA_CMDEX_KEY_Val));

        for (uint32_t spin = 0; spin < command_spin_limit; ++spin) {
            if (ready()) {
                return outcome();
            }
        }
        return NvmError::timed_out;
    }

    /// Which error bit, if any, a just-finished command left behind.
    static NvmError outcome() {
        const uint16_t s = status_bits();
        if ((s & NVMCTRL_STATUS_PROGE_Msk) != 0u) {
            return NvmError::program_error;
        }
        if ((s & NVMCTRL_STATUS_LOCKE_Msk) != 0u) {
            return NvmError::lock_error;
        }
        if ((s & NVMCTRL_STATUS_NVME_Msk) != 0u) {
            return NvmError::nvm_error;
        }
        return NvmError::none;
    }

    /// Point ADDR at a byte address of the given array.
    static void address(NvmArray a, uint32_t addr) {
        NVMCTRL_REGS->NVMCTRL_ADDR = addr_field(a, addr);
    }
    /// ADDR as it stands - also updated by hardware on every page-buffer
    /// load, which is how its encoding was measured.
    static uint32_t address() { return NVMCTRL_REGS->NVMCTRL_ADDR; }

    /// Discard whatever the page buffer holds. The buffer is set to all
    /// ones after every page write, so this is only needed to abandon a
    /// partial load (27.6.4.4).
    static NvmError clear_page_buffer() {
        return command(NVMCTRL_CTRLA_CMD_PBC_Val);
    }

    /// Invalidate every cache line. Commands that change NVM content do
    /// this on their own (27.6.7); the verb exists for the cases nothing
    /// else covers - a debugger write, a future DSU operation.
    static NvmError invalidate_cache() {
        return command(NVMCTRL_CTRLA_CMD_INVALL_Val);
    }

    /// Power reduction, by command. CTRLB.SLEEPPRM does it automatically
    /// on sleep entry; these are the manual halves (27.6.4.7).
    static NvmError enter_power_reduction() {
        return command(NVMCTRL_CTRLA_CMD_SPRM_Val);
    }
    static NvmError exit_power_reduction() {
        return command(NVMCTRL_CTRLA_CMD_CPRM_Val);
    }
    static bool power_reduced() {
        return (status_bits() & NVMCTRL_STATUS_PRM_Msk) != 0u;
    }

    // ---- erase and program ------------------------------------------------

    /**
     * Erase the row containing `addr`, setting all its bits to one.
     *
     * The address may be anywhere in the row (27.6.4.5.1) but this
     * refuses anything that is not row-aligned: a caller that means a
     * different row than it wrote is a bug worth catching, and the media
     * layer above always has the aligned address in hand.
     *
     * A locked region reports `lock_error` and nothing is erased.
     */
    static NvmError erase_row(NvmArray a, uint32_t addr) {
        if (!in_array(a, addr) || (addr % row_size) != 0u) {
            return NvmError::bad_address;
        }
        address(a, addr);
        return command(a == NvmArray::main ? NVMCTRL_CTRLA_CMD_ER_Val
                                           : NVMCTRL_CTRLA_CMD_RWWEEER_Val);
    }

    /**
     * Program one whole page from `src`, which must be exactly
     * `page_size` bytes.
     *
     * The load is 32-bit stores in STRICTLY ASCENDING order - the pattern
     * that is immune to both page-buffer traps (fact 3 in this file's
     * header). The destination pointer is volatile because these stores
     * are the operation: they have no readable effect the optimizer can
     * see, and gcc has been observed on this target sinking exactly such
     * stores past the thing that consumes them.
     *
     * `src` is read byte-wise into a word rather than cast, so an
     * unaligned caller buffer costs a shift instead of a fault: this core
     * has no unaligned word access.
     *
     * The row must already have been erased. Nothing here checks that -
     * flash programming can only clear bits, so writing into a dirty page
     * silently ANDs, which is the caller's contract to keep (and
     * util/nv_heap.hpp's whole design is about keeping it).
     */
    static NvmError program_page(NvmArray a, uint32_t addr,
                                 std::span<const uint8_t> src) {
        if (!in_array(a, addr) || (addr % page_size) != 0u ||
            src.size() != page_size) {
            return NvmError::bad_address;
        }
        if (!ready()) {
            return NvmError::busy;
        }

        const NvmError cleared = clear_page_buffer();
        if (cleared != NvmError::none) {
            return cleared;
        }

        volatile uint32_t* dst = reinterpret_cast<volatile uint32_t*>(addr);
        for (uint32_t i = 0; i < page_size; i += 4u) {
            const uint32_t word =
                static_cast<uint32_t>(src[i]) |
                (static_cast<uint32_t>(src[i + 1u]) << 8) |
                (static_cast<uint32_t>(src[i + 2u]) << 16) |
                (static_cast<uint32_t>(src[i + 3u]) << 24);
            dst[i / 4u] = word;
        }

        address(a, addr);
        return command(a == NvmArray::main ? NVMCTRL_CTRLA_CMD_WP_Val
                                           : NVMCTRL_CTRLA_CMD_RWWEEWP_Val);
    }

    /// Read `dst.size()` bytes from `addr`. Flash is memory-mapped on
    /// both arrays, so this is a copy and cannot fail; erased flash reads
    /// as 0xFF.
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        const volatile uint8_t* p = reinterpret_cast<const volatile uint8_t*>(addr);
        for (uint32_t i = 0; i < dst.size(); ++i) {
            dst[i] = p[i];
        }
    }

    /// One 32-bit word, for the factory areas and for callers that want
    /// no span. Word-aligned by contract - this core cannot do otherwise.
    static uint32_t read_word(uint32_t addr) {
        return *reinterpret_cast<const volatile uint32_t*>(addr);
    }

    // ---- region protection ------------------------------------------------

    /**
     * LOCK: one bit per region, ZERO meaning locked (27.8.9). Loaded from
     * the user row at reset; the two commands below change it only until
     * the next reset.
     */
    static uint16_t locks() { return NVMCTRL_REGS->NVMCTRL_LOCK; }

    static bool region_locked(uint8_t region) {
        return region < region_count && (locks() & (1u << region)) == 0u;
    }

    /// Lock (or unlock) the region containing `addr`, until the next
    /// reset. To change the state a reset restores, the user row has to
    /// be written - which this driver deliberately does not do.
    static NvmError lock_region(uint32_t addr) {
        if (!in_array(NvmArray::main, addr)) {
            return NvmError::bad_address;
        }
        address(NvmArray::main, addr);
        return command(NVMCTRL_CTRLA_CMD_LR_Val);
    }
    static NvmError unlock_region(uint32_t addr) {
        if (!in_array(NvmArray::main, addr)) {
            return NvmError::bad_address;
        }
        address(NvmArray::main, addr);
        return command(NVMCTRL_CTRLA_CMD_UR_Val);
    }
};

// =============================================================================
// Read wait states - NVMCTRL's register, the clock code's question
// =============================================================================

/**
 * CTRLB.RWS. Kept as its own type, and kept under the name samc/clock.hpp
 * already used, because the two sides of this are genuinely split: the
 * REGISTER belongs to NVMCTRL, the QUESTION ("how many at this rate?") is
 * the clock's, and nothing may raise the CPU frequency without answering
 * it first (27.5.2 orders wait states adapted BEFORE a rise and after a
 * fall).
 *
 * Note the field's position: RWS is CTRLB bits 4:1, not 3:0 - the device
 * header's NVMCTRL_CTRLB_RWS_Pos is 1, and it is the authority.
 */
struct FlashWaitStates {
    FlashWaitStates() = delete;

    static uint8_t get() {
        return static_cast<uint8_t>(
            (NVMCTRL_REGS->NVMCTRL_CTRLB & NVMCTRL_CTRLB_RWS_Msk) >>
            NVMCTRL_CTRLB_RWS_Pos);
    }

    static void set(uint8_t rws) {
        NVMCTRL_REGS->NVMCTRL_CTRLB =
            (NVMCTRL_REGS->NVMCTRL_CTRLB & ~NVMCTRL_CTRLB_RWS_Msk) |
            NVMCTRL_CTRLB_RWS(rws);
    }

    /// Wait states the flash needs at `hz` (table 45-41, VDD > 2.7 V: the
    /// conservative column - the 5 V one only buys 1 MHz at 0 WS).
    static constexpr uint8_t for_hz(uint32_t hz) {
        if (hz <= 19'000'000UL) return 0;
        if (hz <= 38'000'000UL) return 1;
        return 2;
    }
};

// =============================================================================
// The read-only factory areas
// =============================================================================

/**
 * The NVM User Row (9.3, table 9-4) as a typed view. This is the fuse row
 * of this family: what it holds is in force from power-on, and a change
 * takes effect only after a reset.
 *
 * READ ONLY HERE. The row survives a chip erase, and it carries the
 * watchdog's power-on ALWAYSON bit and the brown-out level - a wrong word
 * is not undone by reflashing. Writing it is provisioning and wants a
 * bench tool, not an application call.
 *
 * The fields are read from the two 32-bit words the row starts with;
 * bit 32 and above live in word 1, which is why the watchdog window and
 * early-warning fields are computed from it.
 */
struct NvmUserRow {
    uint32_t word0;
    uint32_t word1;

    static NvmUserRow read() {
        return NvmUserRow{Nvm::read_word(Nvm::user_row),
                          Nvm::read_word(Nvm::user_row + 4u)};
    }

    /// BOOTPROT[2:0] - bits 2:0. The FIELD, not the size: 0x7 (the
    /// production setting) means no protection, and each step down
    /// doubles the protected area from two rows.
    constexpr uint8_t bootprot_field() const { return static_cast<uint8_t>(word0 & 0x7u); }
    /// Rows the bootloader section occupies, decoded from table 27-2.
    constexpr uint16_t bootprot_rows() const {
        const uint8_t f = bootprot_field();
        return f == 0x7u ? 0u : static_cast<uint16_t>(2u << (6u - f));
    }
    constexpr uint32_t bootprot_bytes() const {
        return static_cast<uint32_t>(bootprot_rows()) * Nvm::row_size;
    }

    /// EEPROM[2:0] - bits 6:4, the emulation area carved out of the TOP
    /// of the main array (table 27-3). Its rows are writable regardless
    /// of region lock status.
    constexpr uint8_t eeprom_field() const { return static_cast<uint8_t>((word0 >> 4) & 0x7u); }
    constexpr uint16_t eeprom_rows() const {
        const uint8_t f = eeprom_field();
        return f == 0x7u ? 0u : static_cast<uint16_t>(1u << (6u - f));
    }
    constexpr uint32_t eeprom_bytes() const {
        return static_cast<uint32_t>(eeprom_rows()) * Nvm::row_size;
    }

    /// SUPC.BODVDD mirrors, in force from power-on.
    constexpr uint8_t bodvdd_level() const { return static_cast<uint8_t>((word0 >> 8) & 0x3Fu); }
    constexpr bool bodvdd_disabled() const { return ((word0 >> 14) & 0x1u) != 0u; }
    constexpr uint8_t bodvdd_action() const { return static_cast<uint8_t>((word0 >> 15) & 0x3u); }

    /// WDT.CTRLA / WDT.CONFIG mirrors, in force from power-on.
    constexpr bool wdt_enabled() const { return ((word0 >> 26) & 0x1u) != 0u; }
    constexpr bool wdt_always_on() const { return ((word0 >> 27) & 0x1u) != 0u; }
    constexpr uint8_t wdt_period() const { return static_cast<uint8_t>((word0 >> 28) & 0xFu); }
    constexpr uint8_t wdt_window() const { return static_cast<uint8_t>(word1 & 0xFu); }
    constexpr uint8_t wdt_ew_offset() const { return static_cast<uint8_t>((word1 >> 4) & 0xFu); }
};

/**
 * The NVM Software Calibration Area (9.4, table 9-5) at 0x00806020: eight
 * bytes of factory values that the application is expected to copy into
 * the peripherals that need them. None of them can be computed and none
 * are loaded automatically - a driver that skips this runs an uncalibrated
 * converter or oscillator.
 *
 * The area CANNOT be written. Every consumer is a future driver (ADC0,
 * ADC1, OSC32KCTRL, OSCCTRL), which is why this view lands with the NVM
 * driver rather than with any of them.
 */
struct NvmCalibration {
    uint32_t word0;
    uint32_t word1;

    static NvmCalibration read() {
        return NvmCalibration{Nvm::read_word(Nvm::software_calibration),
                              Nvm::read_word(Nvm::software_calibration + 4u)};
    }

    constexpr uint8_t adc0_biasrefbuf() const { return static_cast<uint8_t>(word0 & 0x7u); }
    constexpr uint8_t adc0_biascomp() const { return static_cast<uint8_t>((word0 >> 3) & 0x7u); }
    constexpr uint8_t adc1_biasrefbuf() const { return static_cast<uint8_t>((word0 >> 6) & 0x7u); }
    constexpr uint8_t adc1_biascomp() const { return static_cast<uint8_t>((word0 >> 9) & 0x7u); }
    /// OSC32K CALIB - bits 18:12.
    constexpr uint8_t osc32k_calib() const { return static_cast<uint8_t>((word0 >> 12) & 0x7Fu); }
    /// OSCCTRL CAL48M for VDD 3.6..5.5 V - bits 40:19, so it straddles
    /// the word boundary.
    constexpr uint32_t cal48m_5v() const {
        return ((word0 >> 19) | (word1 << 13)) & 0x3FFFFFUL;
    }
    /// OSCCTRL CAL48M for VDD 2.7..3.6 V - bits 62:41.
    constexpr uint32_t cal48m_3v3() const { return (word1 >> 9) & 0x3FFFFFUL; }
};

/**
 * The NVM Temperature Calibration Area (9.5, table 9-6) at 0x00806030 -
 * SAM C21 only, and the TSENS driver's to consume. Read-only, like its
 * neighbour.
 */
struct NvmTemperatureCalibration {
    uint32_t word0;
    uint32_t word1;

    static NvmTemperatureCalibration read() {
        return NvmTemperatureCalibration{
            Nvm::read_word(Nvm::temperature_calibration),
            Nvm::read_word(Nvm::temperature_calibration + 4u)};
    }

    constexpr uint8_t tsens_tcal() const { return static_cast<uint8_t>(word0 & 0x3Fu); }
    constexpr uint8_t tsens_fcal() const { return static_cast<uint8_t>((word0 >> 6) & 0x3Fu); }
    /// TSENS GAIN - bits 35:12, straddling the word boundary.
    constexpr uint32_t tsens_gain() const {
        return ((word0 >> 12) | (word1 << 20)) & 0xFFFFFFUL;
    }
    /// TSENS OFFSET - bits 59:36.
    constexpr uint32_t tsens_offset() const { return (word1 >> 4) & 0xFFFFFFUL; }
};

/**
 * The die's 128-bit serial number (9.6): four words that are NOT
 * contiguous - word 0 sits apart from words 1..3. Factory-programmed, and
 * the reason a SAM board needs no written identity label where an AVR one
 * does.
 *
 * "The uniqueness of the serial number is guaranteed only when using all
 * 128 bits" - so a program that wants a short id must hash all four, not
 * take the first.
 */
struct DeviceSerial {
    uint32_t word[4];

    static DeviceSerial read() {
        return DeviceSerial{{Nvm::read_word(0x0080A00CUL),
                             Nvm::read_word(0x0080A040UL),
                             Nvm::read_word(0x0080A044UL),
                             Nvm::read_word(0x0080A048UL)}};
    }
};

} // namespace brio
