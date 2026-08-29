/*
 * dsu.hpp
 *
 * The SAM C21 Device Service Unit (DS60001479M ch. 13): the block a
 * debug probe talks to before the CPU is even running, and the one
 * peripheral on this device that answers the question "what chip is
 * this?".
 *
 * WHAT IT IS FOR HERE. Three of its services are useful to code running
 * ON the device rather than to a probe attached to it, and those are
 * what this header builds:
 *
 *  1. DEVICE IDENTIFICATION. DID decodes into processor, family,
 *     series, die, revision and device-select - the number every errata
 *     row is indexed by. Board identity on this family comes free: this
 *     register plus the factory 128-bit serial samc/nvm.hpp already
 *     reads (`DeviceSerial`) is a label no chip erase can remove, where
 *     an AVR-Dx board has to be GIVEN one by hand in its USERROW.
 *  2. A HARDWARE CRC32 OVER ANY MEMORY THE BUS MATRIX REACHES. The
 *     engine walks flash or SRAM with no CPU in the loop and no code
 *     size spent on a table; the polynomial is the industry-standard
 *     reflected 0xEDB88320, so the answer matches any host-side tool
 *     once complemented.
 *  3. MBIST, the March LR memory self-test, which exists here because
 *     IEC 60730 Class B asks for one. IT DESTROYS WHAT IT TESTS - the
 *     algorithm's first phase writes the whole range to zero - so the
 *     only honest way to run it from firmware is over a buffer the
 *     caller owns and has finished with. That is what `mbist()`
 *     documents and what the bench suite does.
 *
 * WHAT IS DELIBERATELY NOT HERE: CHIP ERASE. CTRL.CE erases the whole
 * flash array including the EEPROM emulation area and clears the
 * security bit; a firmware-callable verb for it would be a verb for
 * destroying the running program, and this framework's AVR half made
 * the same call about NVMCTRL's CHER (see docs/avrdx/nvm.md). The bit
 * is named in docs/samc/dsu.md and not exposed.
 *
 * THE PROTECTION SURPRISE, and it is the first thing this driver had to
 * deal with. PAC.STATUSB's RESET VALUE IS 0x00000002 (11.7.11): the DSU
 * comes out of reset ALREADY WRITE-PROTECTED, alone among the
 * peripherals of this device, and table 12-3's "Prot at Reset" column
 * says Y for this row and N for every other. So a bare store to
 * DSU.ADDR does nothing at all and raises PAC.INTFLAGB.DSU instead;
 * `init()` therefore clears the protection through samc/pac.hpp, and
 * `release()` puts it back exactly as reset left it.
 *
 * THE INTERNAL AND EXTERNAL ADDRESS RANGES. 13.9: the first 0x100 bytes
 * of the register map are mirrored at 0x100, so the silicon can tell a
 * CPU access from a debugger one and restrict the latter when the device
 * is protected by the NVM security bit. Everything here uses the
 * INTERNAL range (the device header's `DSU_REGS`), which 13.12.2.3 is
 * explicit is the right choice for code running on the CPU.
 *
 * TWO REGISTERS THE CHAPTER DOES NOT DESCRIBE. The device header
 * declares `DSU_STATUSC` at offset 0x03 (a three-bit STATE field) and
 * `DSU_DCFG[2]` at 0xF0, neither of which appears in the register
 * summary of 13.13 - offset 0x03 is printed as "Reserved" there. They
 * are exposed here as raw reads only, with no claim about what they
 * mean, because the header is the authority on what exists and the
 * datasheet is the authority on what it does - and here the datasheet
 * says nothing.
 *
 * NO ERRATA. DS80000740S has no DSU section at all. The one item that
 * touches this chapter's subject matter is 1.8.15 (Device): the Program
 * and Debug Interface Disable lock of 13.10 is NOT AVAILABLE on E/G/J
 * silicon up to and including revision F - which is a feature this
 * header does not build in any case.
 *
 * NOT BUILT (docs/samc/dsu.md carries the list): chip erase, the DAP
 * security filter and everything else that is a debugger's business
 * rather than the CPU's; the DMA connection of the two debug
 * communication channels; and the CoreSight ROM beyond reading its
 * entries back.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/device_tables.hpp"
#include "samc/pac.hpp"

namespace brio {

/// STATUSA (13.14.2). Every bit is write-one-to-clear.
struct DsuStatus {
    static constexpr uint8_t done = DSU_STATUSA_DONE_Msk;
    static constexpr uint8_t crstext = DSU_STATUSA_CRSTEXT_Msk;
    static constexpr uint8_t bus_error = DSU_STATUSA_BERR_Msk;
    static constexpr uint8_t fail = DSU_STATUSA_FAIL_Msk;
    static constexpr uint8_t protection_error = DSU_STATUSA_PERR_Msk;
    static constexpr uint8_t all = static_cast<uint8_t>(
        done | crstext | bus_error | fail | protection_error);
};

/// ADDR.AMOD (13.14.4): a two-bit field whose meaning depends on which
/// command is running - the chapter gives it two different tables
/// (13-3 for CRC32, 13-5 for MBIST) and this enumeration keeps both
/// namings rather than inventing a third.
enum class DsuAccessMode : uint8_t {
    /// CRC32 from the external range: the full flash array. MBIST:
    /// exit-on-error. Zero either way, and the value a CPU-side caller
    /// always wants.
    array_or_exit_on_error = 0,
    /// CRC32 from the external range: the EEPROM emulation area.
    /// MBIST: pause-on-error.
    eeprom_or_pause_on_error = 1,
};

/// DID (13.14.9), decoded. Nothing here interprets the numbers into
/// marketing names - that mapping lives in the ordering information and
/// changes with every new part - but the fields are what every errata
/// row and every device-pack lookup is keyed by.
struct DsuDeviceId {
    uint32_t raw;
    uint8_t processor;   ///< DID.PROCESSOR, 1 = Cortex-M0+
    uint8_t family;      ///< DID.FAMILY, 2 = "5V Industrial" (the SAM C)
    uint8_t series;      ///< DID.SERIES, 1 = Cortex-M0+ with CAN (C21)
    uint8_t die;         ///< DID.DIE
    uint8_t revision;    ///< DID.REVISION, 0 = rev A, 1 = rev B, ...
    uint8_t devsel;      ///< DID.DEVSEL, the part within the series

    /// The silicon revision as the errata document spells it: revision
    /// 0 is 'A', 5 is 'F'. THE ERRATA MATRIX IS INDEXED BY THIS LETTER,
    /// so it is the one derived value worth computing here.
    constexpr char revision_letter() const {
        return static_cast<char>('A' + revision);
    }
};

/// What an MBIST run reports. `failed` false and `done` true is a pass;
/// on a failure the address and data registers locate the bit, decoded
/// per figure 13-6.
struct DsuMbistResult {
    bool done = false;
    bool failed = false;
    bool bus_error = false;
    bool protection_error = false;
    uint32_t address = 0;   ///< ADDR at the moment of the failure
    uint32_t data = 0;      ///< DATA at the moment of the failure
    uint8_t phase = 0;      ///< DATA[11:8], table 13-4
    uint8_t bit_index = 0;  ///< DATA[7:0], the failing bit within the word
};

/**
 * The Device Service Unit as a monostate resource.
 */
struct Dsu {
    Dsu() = delete;

    /// PAC.WRCTRL.PERID. Published like every id in this stratum - and
    /// USED here, unlike anywhere else, because this peripheral is the
    /// one that comes up protected.
    static constexpr uint16_t pac_id = dsu_pac_id();   // 33

    /// The DSU has no interrupt line (13.5.5 "Not applicable") and no
    /// events (13.5.6). Its only asynchronous signal is STATUSA.DONE,
    /// which is polled.

    static dsu_registers_t& regs() { return *DSU_REGS; }

    // ---- clocks and protection ---------------------------------------------

    /// Both of the DSU's clocks are ON out of reset (table 12-3: AHB
    /// index 3 and APB-B index 1, "Enabled at Reset" Y for both).
    static void bus_clock(bool on) {
        Mclk::ahb(MCLK_AHBMASK_DSU_Msk, on);
        Mclk::apb_b(MCLK_APBBMASK_DSU_Msk, on);
    }

    /**
     * Make the block writable: clocks on, and the reset-time PAC write
     * protection cleared.
     *
     * THE UNPROTECT IS CONDITIONAL, and that is the double-write rule of
     * 11.5.2.6 rather than caution: clearing protection that is already
     * clear is itself an error and would set PAC.INTFLAGB.DSU.
     *
     * Returns whether the block is writable afterwards - false if the
     * protection is LOCKED, which nothing here does but a safety-minded
     * application might.
     */
    static bool init() {
        bus_clock(true);
        if (Pac::is_protected(pac_id)) {
            (void)Pac::unprotect(pac_id);
        }
        return !Pac::is_protected(pac_id);
    }

    /// Put the block back the way a reset leaves it: protected. The
    /// clocks are left ON, because a reset leaves them on too.
    static void release() {
        clear_status(DsuStatus::all);
        if (!Pac::is_protected(pac_id)) {
            (void)Pac::protect(pac_id);
        }
    }

    // ---- identity -----------------------------------------------------------

    static uint32_t did_raw() { return DSU_REGS->DSU_DID; }

    static DsuDeviceId device_id() {
        const uint32_t v = did_raw();
        return DsuDeviceId{
            v,
            static_cast<uint8_t>((v & DSU_DID_PROCESSOR_Msk) >> DSU_DID_PROCESSOR_Pos),
            static_cast<uint8_t>((v & DSU_DID_FAMILY_Msk) >> DSU_DID_FAMILY_Pos),
            static_cast<uint8_t>((v & DSU_DID_SERIES_Msk) >> DSU_DID_SERIES_Pos),
            static_cast<uint8_t>((v & DSU_DID_DIE_Msk) >> DSU_DID_DIE_Pos),
            static_cast<uint8_t>((v & DSU_DID_REVISION_Msk) >> DSU_DID_REVISION_Pos),
            static_cast<uint8_t>((v & DSU_DID_DEVSEL_Msk) >> DSU_DID_DEVSEL_Pos),
        };
    }

    // ---- status -------------------------------------------------------------

    static uint8_t status() { return DSU_REGS->DSU_STATUSA; }
    static void clear_status(uint8_t mask = DsuStatus::all) {
        DSU_REGS->DSU_STATUSA = mask;
    }
    static bool done() { return (status() & DsuStatus::done) != 0u; }
    static bool bus_error() { return (status() & DsuStatus::bus_error) != 0u; }
    static bool failed() { return (status() & DsuStatus::fail) != 0u; }
    static bool protection_error() {
        return (status() & DsuStatus::protection_error) != 0u;
    }

    /// STATUSA.CRSTEXT: set when a cold-plugged debugger is holding the
    /// CPU in the extended reset phase. Code that can read this is by
    /// definition running, so it can only ever read it as clear - the
    /// verb exists for completeness and for the one thing it CAN say,
    /// which is that nothing is holding the CPU now.
    static bool cpu_reset_extended() {
        return (status() & DsuStatus::crstext) != 0u;
    }
    static void release_cpu_reset() { DSU_REGS->DSU_STATUSA = DsuStatus::crstext; }

    static uint8_t status_b() { return DSU_REGS->DSU_STATUSB; }
    /// STATUSB.PROT: the device is protected by the NVM security bit.
    static bool device_protected() {
        return (status_b() & DSU_STATUSB_PROT_Msk) != 0u;
    }
    /// STATUSB.DBGPRES: a debugger probe was detected, by cold- or
    /// hot-plugging. NEVER CLEARED once set (13.14.3), so this is a
    /// since-power-on fact and not a now fact.
    static bool debugger_present() {
        return (status_b() & DSU_STATUSB_DBGPRES_Msk) != 0u;
    }
    /// STATUSB.HPE: hot-plugging still available. Cleared for good (until
    /// a power-on or external reset) the moment the SWCLK pad's function
    /// is changed away from the debug system.
    static bool hot_plugging_enabled() {
        return (status_b() & DSU_STATUSB_HPE_Msk) != 0u;
    }
    static bool dcc_dirty(uint8_t n) {
        return n < 2u && (status_b() & (DSU_STATUSB_DCCD0_Msk << n)) != 0u;
    }

    /// STATUSC, WHICH CHAPTER 13 DOES NOT DESCRIBE: the device header
    /// declares an 8-bit read-only register at offset 0x03 with a
    /// three-bit STATE field, where 13.13's register summary prints
    /// "Reserved". Returned raw, with no interpretation, because there
    /// is no document here to interpret it against.
    static uint8_t status_c_raw() { return DSU_REGS->DSU_STATUSC; }

    /// DCFG0/DCFG1 at offset 0xF0, likewise absent from chapter 13.
    static uint32_t dcfg_raw(uint8_t n) {
        return n < 2u ? DSU_REGS->DSU_DCFG[n] : 0u;
    }

    // ---- the debug communication channels -----------------------------------
    //
    // Two 32-bit mailboxes shared with a debugger, readable and writable
    // from both sides even when the device is security-protected, with a
    // dirty bit each that sets on write and clears on read (13.12.4).
    // NOTE 13.12.4's warning: they are shared with the MBIST logic, so
    // they must not be used while a memory test runs.

    static uint32_t dcc(uint8_t n) { return n < 2u ? DSU_REGS->DSU_DCC[n] : 0u; }
    static void dcc(uint8_t n, uint32_t value) {
        if (n < 2u) {
            DSU_REGS->DSU_DCC[n] = value;
        }
    }

    // ---- the shared command interface ---------------------------------------
    //
    // 13.12.1: ADDR, LENGTH and DATA are shared by CRC32 and MBIST, and
    // a command is issued by one write to CTRL. While a command runs,
    // further commands are DISCARDED - so every caller must wait for
    // STATUSA.DONE before issuing another, which is what the verbs below
    // do rather than leaving it to the caller.

    /// ADDR takes a BYTE address in bits 31:2 and AMOD in bits 1:0; the
    /// address must be word-aligned, which the mask enforces.
    static void set_address(uint32_t byte_address, DsuAccessMode mode) {
        DSU_REGS->DSU_ADDR = (byte_address & ~3UL) |
                             static_cast<uint32_t>(mode);
    }

    /**
     * LENGTH, and the one place two readings of chapter 13 are possible.
     *
     * 13.14.5 names the field "LENGTH[29:0] Length in words", and the
     * field sits at bits 31:2 - so the REGISTER value is four times the
     * word count, i.e. a byte length, and what the field reads back is
     * the number of words. This verb takes WORDS and writes words x 4,
     * which the bench confirms by matching the engine's CRC32 against a
     * software one over exactly that many words (test_samc_debug letter
     * e).
     */
    static void set_length_words(uint32_t words) {
        DSU_REGS->DSU_LENGTH = words << 2;
    }

    static uint32_t address_raw() { return DSU_REGS->DSU_ADDR; }
    static uint32_t length_raw() { return DSU_REGS->DSU_LENGTH; }
    static uint32_t data() { return DSU_REGS->DSU_DATA; }
    static void data(uint32_t v) { DSU_REGS->DSU_DATA = v; }

    /// CTRL.SWRST. Cancels a running CRC32 or MBIST (13.12.3.1) and is
    /// the only way to stop one.
    static void reset_module() { DSU_REGS->DSU_CTRL = DSU_CTRL_SWRST_Msk; }

    /// Wait for STATUSA.DONE, bounded. False means the bound ran out -
    /// never that the command failed, which is what STATUSA's other bits
    /// are for.
    static bool wait_done(uint32_t spins = 20'000'000UL) {
        while (!done() && spins-- != 0u) {
        }
        return done();
    }

    // ---- CRC32 ---------------------------------------------------------------

    /**
     * Whether a shared-command range can be issued at all: a non-empty,
     * word-aligned one. Both engines walk WORDS through the bus matrix,
     * so an odd address is not slow but meaningless, and a zero length
     * is a command with nothing to do that would still set DONE.
     */
    static constexpr bool range_valid(uint32_t byte_address, uint32_t words) {
        return words != 0u && (byte_address & 3UL) == 0u;
    }

    /**
     * CRC32 over `words` 32-bit words starting at `byte_address`.
     *
     * The seed defaults to 0xFFFFFFFF, which with the returned value
     * COMPLEMENTED gives the industry-standard CRC-32 over the same
     * bytes (13.12.3: the polynomial is 0xEDB88320 in reversed
     * representation). Passing a previous NON-complemented result as the
     * seed chains two ranges into one checksum - which is why
     * `crc32_raw()` exists beside this.
     *
     * Nothing is returned when the wait ran out or when STATUSA.BERR
     * says the engine hit an address the bus matrix would not serve;
     * 13.12.3.2 requires that check and a checksum over a bus error is
     * not a checksum.
     */
    static std::optional<uint32_t> crc32_raw(uint32_t byte_address, uint32_t words,
                                             uint32_t seed = 0xFFFFFFFFUL,
                                             uint32_t spins = 20'000'000UL) {
        if (!range_valid(byte_address, words)) {
            return std::nullopt;
        }
        clear_status(DsuStatus::all);
        set_address(byte_address, DsuAccessMode::array_or_exit_on_error);
        set_length_words(words);
        data(seed);
        DSU_REGS->DSU_CTRL = DSU_CTRL_CRC_Msk;
        if (!wait_done(spins)) {
            return std::nullopt;
        }
        const bool berr = bus_error();
        const uint32_t result = data();
        clear_status(DsuStatus::all);
        if (berr) {
            return std::nullopt;
        }
        return result;
    }

    /// The same, complemented - the number a host-side CRC-32 of the
    /// same bytes produces.
    static std::optional<uint32_t> crc32(uint32_t byte_address, uint32_t words,
                                         uint32_t seed = 0xFFFFFFFFUL,
                                         uint32_t spins = 20'000'000UL) {
        const auto raw = crc32_raw(byte_address, words, seed, spins);
        if (!raw) {
            return std::nullopt;
        }
        return ~*raw;
    }

    // ---- MBIST ---------------------------------------------------------------

    /**
     * Run the March LR memory self-test over `words` words at
     * `byte_address`.
     *
     * IT DESTROYS THE RANGE. Phase 0 writes every bit to zero and the
     * test ends having written the whole range again; there is no
     * save-and-restore anywhere in the algorithm. The caller must own
     * the memory and be finished with it - a range overlapping the
     * stack, a live global or the vector table takes the program down
     * with it, and this header cannot check that for you.
     *
     * `pause_on_error` selects AMOD 1, where the state machine stops at
     * the first bad bit and waits for STATUSA.FAIL to be written back
     * before resuming. This verb does not resume it: it reports the
     * failure and leaves the engine parked, because a caller that wants
     * every failing bit wants a loop of its own.
     */
    static DsuMbistResult mbist(uint32_t byte_address, uint32_t words,
                                bool pause_on_error = false,
                                uint32_t spins = 20'000'000UL) {
        DsuMbistResult r{};
        if (!range_valid(byte_address, words)) {
            return r;
        }
        clear_status(DsuStatus::all);
        set_address(byte_address, pause_on_error
                                      ? DsuAccessMode::eeprom_or_pause_on_error
                                      : DsuAccessMode::array_or_exit_on_error);
        set_length_words(words);
        DSU_REGS->DSU_CTRL = DSU_CTRL_MBIST_Msk;

        // In pause-on-error mode a failure raises FAIL WITHOUT DONE
        // (13.12.5), so both have to be watched.
        while (spins-- != 0u) {
            const uint8_t s = status();
            if ((s & (DsuStatus::done | DsuStatus::fail)) != 0u) {
                break;
            }
        }
        const uint8_t s = status();
        r.done = (s & DsuStatus::done) != 0u;
        r.failed = (s & DsuStatus::fail) != 0u;
        r.bus_error = (s & DsuStatus::bus_error) != 0u;
        r.protection_error = (s & DsuStatus::protection_error) != 0u;
        if (r.failed) {
            r.address = address_raw();
            r.data = data();
            r.phase = static_cast<uint8_t>((r.data >> 8) & 0x0Fu);
            r.bit_index = static_cast<uint8_t>(r.data & 0xFFu);
        }
        reset_module();
        clear_status(DsuStatus::all);
        return r;
    }

    // ---- the CoreSight ROM table --------------------------------------------
    //
    // 13.11: a system-level ARM CoreSight ROM at offset 0x1000, whose
    // conceptual 64-bit peripheral ID identifies the part as a SAM
    // device implementing a DSU (PARTNUM 0xCD0, JEP-106 continuation 0,
    // ID code 0x1F). Read-only, and read here so the identification path
    // a probe uses can be checked from the inside.

    static uint32_t rom_entry(uint8_t n) {
        return n == 0u ? DSU_REGS->DSU_ENTRY0
                       : (n == 1u ? DSU_REGS->DSU_ENTRY1 : 0u);
    }
    /// ENTRY.EPRES, whose two sentences in 13.14.10 contradict each
    /// other word for word (both begin "if the device is not
    /// protected"). Returned as the bit it is.
    static bool rom_entry_present(uint8_t n) {
        return (rom_entry(n) & DSU_ENTRY0_EPRES_Msk) != 0u;
    }
    /// The component's base address, relative to the ROM table's own.
    static uint32_t rom_entry_offset(uint8_t n) {
        return rom_entry(n) & 0xFFFFF000UL;
    }
    static uint32_t rom_end() { return DSU_REGS->DSU_END; }
    static uint32_t rom_memtype() { return DSU_REGS->DSU_MEMTYPE; }

    /// PID0..PID7 as the flat array the CoreSight numbering does not
    /// give them: index 0..3 are PID0..PID3, 4..7 are PID4..PID7.
    static uint32_t rom_pid(uint8_t n) {
        switch (n) {
            case 0: return DSU_REGS->DSU_PID0;
            case 1: return DSU_REGS->DSU_PID1;
            case 2: return DSU_REGS->DSU_PID2;
            case 3: return DSU_REGS->DSU_PID3;
            case 4: return DSU_REGS->DSU_PID4;
            case 5: return DSU_REGS->DSU_PID5;
            case 6: return DSU_REGS->DSU_PID6;
            case 7: return DSU_REGS->DSU_PID7;
            default: return 0;
        }
    }
    static uint32_t rom_cid(uint8_t n) {
        switch (n) {
            case 0: return DSU_REGS->DSU_CID0;
            case 1: return DSU_REGS->DSU_CID1;
            case 2: return DSU_REGS->DSU_CID2;
            case 3: return DSU_REGS->DSU_CID3;
            default: return 0;
        }
    }

    /// PARTNUM, the twelve bits table 13-2 says contain 0xCD0 to
    /// indicate that a DSU is present: PID0's PARTNBL plus PID1's
    /// PARTNBH.
    static uint16_t rom_partnum() {
        const uint32_t p0 = rom_pid(0) & 0xFFu;
        const uint32_t p1 = rom_pid(1) & 0x0Fu;
        return static_cast<uint16_t>(p0 | (p1 << 8));
    }
};

} // namespace brio
