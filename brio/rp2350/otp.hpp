/*
 * otp.hpp
 *
 * THE ONE-TIME PROGRAMMABLE ARRAY, READ SIDE ONLY (datasheet chapter 13,
 * with 4.5 and 10.8): 4096 rows of 24 bits, of which the first rows are
 * what the factory wrote about this die and the rest is what an owner
 * has put there. A block the RP2040 never had.
 *
 * THIS FILE HAS NO WRITE PATH, AND THAT IS A DECISION AND NOT AN
 * OVERSIGHT. Every bit of this array goes from zero to one exactly once
 * and never back. Among those bits are CRIT1's SECURE_BOOT_ENABLE,
 * DEBUG_DISABLE and SECURE_DEBUG_DISABLE and CRIT0's two architecture
 * disables (13.4) - each of which, once set, permanently removes a
 * debug port, a processor architecture or the ability to run unsigned
 * code on the part that carries it. A driver that can program OTP is a
 * driver that can brick the board it is being developed on, with no way
 * back and no second try. So: no SBPI bridge, no bootrom `otp_access`
 * (its write flag is not a parameter of anything here because the call
 * is not here at all), and NO SOFT LOCK either - SW_LOCKn is read and
 * decoded, never written, because a lock only ever advances and the one
 * that costs nothing to set costs a reset to undo. The programming side
 * of chapter 13 belongs to whatever tool provisions a device, and that
 * tool is not a brio program.
 *
 * THE FOUR READ WINDOWS (13.1). The block's control registers live at
 * OTP_BASE; the DATA lives in a second 64 kB region above them, split
 * into four aliases by two address bits:
 *
 *   0x40130000  ECC          a 16-bit datum per row, error-corrected in
 *                            hardware, read as a HALFWORD at 2 x row
 *   0x40134000  raw          the row's 24 bits as a word at 4 x row, ECC
 *                            and bit-repair bypassed
 *   0x40138000  ECC guarded  the same data, plus hardware consistency
 *   0x4013c000  raw guarded  checks - and a BUS FAULT instead of a
 *                            pattern of ones where a read is refused
 *
 * An UNGUARDED read that the page locks refuse returns all-ones and no
 * fault, which is indistinguishable from a row that genuinely reads
 * all-ones; a GUARDED read faults instead, which is safer and is what
 * the bootrom uses for boot flags, but a fault is not a value a function
 * can return. So this file offers both, and offers `read()` above them:
 * it asks the page's lock FIRST and only then touches the array, so a
 * caller gets an answer or an empty optional and never a fault.
 *
 * ECC, DECODED IN SOFTWARE (13.6). An ECC row carries 16 bits of data,
 * a 6-bit modified Hamming code and 2 bits of bit-repair-by-polarity.
 * The hardware corrects transparently and tells nobody; the only way for
 * a program to learn that a row needed correcting - or that it is beyond
 * correcting - is to read the RAW row and do the arithmetic. 13.6.2
 * publishes that arithmetic as a six-entry parity table, and
 * `otp_ecc_decode()` below is it: constexpr, so a test can flip a bit in
 * a known codeword and check the verdict with no silicon at all.
 *
 * REDUNDANCY WITHOUT ECC (13.10). A row whose bits are programmed at
 * different times cannot carry ECC, because the code covers all sixteen
 * data bits at once. Those rows are replicated instead: three copies
 * with a majority vote for most of them, EIGHT copies with a
 * THREE-OF-EIGHT vote for the critical flags, which is deliberately
 * biased towards reading a flag as SET. `otp_vote3()` and `otp_vote8()`
 * are those two rules.
 *
 * ARCHSEL_STATUS LIVES HERE. Which architecture each core is currently
 * running is an OTP-block register (offset 0x15c), not a power-manager
 * one: the critical flags that force an architecture are read by this
 * block's own power-up state machine, so the register that reports the
 * result sits beside them. It is the one register of this chapter a
 * program on either half is likely to read every day, and it is why this
 * document's chapter and the platform's overlap.
 *
 * THE CLOCK. The OTP subsystem runs from its own ring oscillator during
 * the power-up sequence and switches to clk_ref before any software
 * runs; 13.3 states that clk_ref MUST NOT EXCEED 25 MHz while the OTP is
 * accessed. On this stratum's boards clk_ref is the 12 MHz crystal, so
 * the constraint is met by construction; `clk_ref_max_hz` states it so
 * that a program which moves clk_ref can check.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <optional>

#include "rp2350/device.hpp"

namespace brio {

// =============================================================================
// The geometry (13.1, 13.5)
// =============================================================================

/// Rows of 24 bits in the array.
inline constexpr uint16_t otp_row_count = 4096;
/// Rows per page: the unit the locks work in.
inline constexpr uint16_t otp_page_rows = 64;
/// Pages, therefore.
inline constexpr uint8_t otp_page_count = 64;
/// What the ECC windows expose: two bytes a row over the first 8 kB.
inline constexpr uint32_t otp_ecc_bytes = 8192;

/// 13.3: the OTP may not be accessed with clk_ref above this.
inline constexpr uint32_t otp_clk_ref_max_hz = 25'000'000;

/// The four read windows. The bases the device header names are the two
/// unguarded ones; 13.1 says bit 15 of the address selects guarded, so
/// the other two are derived rather than spelled.
inline constexpr uint32_t otp_guarded_offset = 0x8000u;
inline constexpr uint32_t otp_ecc_window = OTP_DATA_BASE;
inline constexpr uint32_t otp_raw_window = OTP_DATA_RAW_BASE;
inline constexpr uint32_t otp_ecc_guarded_window = OTP_DATA_BASE + otp_guarded_offset;
inline constexpr uint32_t otp_raw_guarded_window = OTP_DATA_RAW_BASE + otp_guarded_offset;

/// What an unguarded read returns when the locks refuse it - and what a
/// row that has been programmed solid also returns, which is the whole
/// reason the guarded windows exist.
inline constexpr uint16_t otp_ecc_refused = 0xFFFFu;
inline constexpr uint32_t otp_raw_refused = 0x00FFFFFFu;

// =============================================================================
// The locks (13.5), read and decoded
// =============================================================================

/// The three states a page's permissions can reach, in the order they
/// advance (13.5.1). The encoding is a two-bit thermometer, so 2 is not
/// a value the hardware produces.
enum class OtpLock : uint8_t {
    read_write = 0,
    read_only = 1,
    inaccessible = 3,
};

/// SW_LOCKn's two-bit field as a state. A 2 would be half-way up the
/// thermometer and is read as the step it has reached.
constexpr OtpLock otp_lock_of(uint32_t bits) {
    return bits == 0u ? OtpLock::read_write
                      : (bits >= 3u ? OtpLock::inaccessible : OtpLock::read_only);
}

/// One page's software lock register: the two security domains advance
/// independently (13.5.1), and both fields are world-readable.
struct OtpPageLock {
    OtpLock secure = OtpLock::read_write;
    OtpLock non_secure = OtpLock::read_write;
};

// =============================================================================
// ECC and bit repair (13.6), in software
// =============================================================================

/// 13.6.2's parity table: six masks over the 22-bit codeword (16 data
/// bits and the five Hamming bits), each defining one parity bit. The
/// sixth covers everything below it, which is what makes the code
/// single-error-correcting and double-error-detecting.
inline constexpr std::array<uint32_t, 6> otp_ecc_parity_masks = {
    0b0000001010110101011011u,
    0b0000000011011001101101u,
    0b0000001100011110001110u,
    0b0000000000011111110000u,
    0b0000001111100000000000u,
    0b0111111111111111111111u,
};

/// Even parity of a word: 1 when an odd number of bits is set.
constexpr uint32_t otp_even_parity(uint32_t value) {
    uint32_t p = 0;
    while (value != 0u) {
        p ^= value & 1u;
        value >>= 1;
    }
    return p;
}

/**
 * The six parity bits a 16-bit datum carries in bits 21:16 of its row.
 * Each is computed over the datum AND the parity bits below it, in
 * order, exactly as 13.6.2's listing accumulates them - which is why
 * this is a loop over a growing word and not six independent sums.
 */
constexpr uint8_t otp_ecc_parity(uint16_t data) {
    uint32_t p = data;
    for (size_t i = 0; i < otp_ecc_parity_masks.size(); ++i) {
        p |= otp_even_parity(p & otp_ecc_parity_masks[i]) << (16 + i);
    }
    return static_cast<uint8_t>((p >> 16) & 0x3Fu);
}

/**
 * THE SIX CHECKS over a 22-bit row as it was READ (data in 15:0, the six
 * parity bits in 21:16), as one value: bits 4:0 are the Hamming
 * syndrome, bit 5 is the overall parity check. Zero means the row is
 * consistent.
 *
 * WHY THE SIXTH CHECK IS NOT THE SIXTH ENCODER OUTPUT. 13.6.2 says the
 * decoder "recalculates the six parity bits based on the value read from
 * the OTP row" and XORs them with the stored six. For the five Hamming
 * bits that is unambiguous: their masks cover data bits only, so
 * recomputing them from the sixteen data bits is right. The SIXTH mask
 * covers bits 0..20 - the data AND the five Hamming bits - so
 * "recalculating" it from the data alone and the RECOMPUTED Hamming bits
 * gives a check that is blind to a whole class of single-bit errors
 * (a flip in data bit 3 of one row, or in a stored parity bit, comes out
 * with the syndrome's top bit CLEAR and would be filed as
 * uncorrectable). The sixth check is the parity of the WHOLE RECEIVED
 * row, which is what makes the code single-error-correcting: any odd
 * number of flips sets it. The family check exercises all twenty-two
 * single flips and all two hundred and thirty-one double ones against
 * this definition.
 */
constexpr uint8_t otp_ecc_checks(uint32_t codeword22) {
    const uint16_t data = static_cast<uint16_t>(codeword22 & 0xFFFFu);
    const uint8_t stored = static_cast<uint8_t>((codeword22 >> 16) & 0x3Fu);
    const uint8_t hamming = static_cast<uint8_t>((stored ^ otp_ecc_parity(data)) & 0x1Fu);
    const uint32_t overall = otp_even_parity(codeword22 & 0x3FFFFFu);
    return static_cast<uint8_t>(hamming | (overall << 5));
}

/// What the software decoder made of a row.
enum class OtpEcc : uint8_t {
    ok,             ///< the stored parity matched
    corrected,      ///< one bit had flipped; `data` is the repaired value
    uncorrectable,  ///< an even number of bits differ: detected, not repairable
};

/// A row as software reads it: the raw 24 bits, what the data means once
/// bit repair and ECC have had their say, and how much repairing that
/// took.
struct OtpRow {
    uint32_t raw = 0;             ///< the 24 bits as the raw window gave them
    uint16_t data = 0;            ///< the 16-bit datum, corrected where it could be
    OtpEcc ecc = OtpEcc::ok;
    uint8_t corrected_bit = 0;    ///< which of the 22 codeword bits, when `corrected`
    bool inverted = false;        ///< bit repair by polarity had inverted the row
};

/**
 * 13.6 in software, over a row read through a RAW window: bit repair
 * first (both of bits 23:22 set means the row was stored inverted), then
 * the modified Hamming code.
 *
 * The verdict follows 13.6.2: a syndrome of zero is no error, a syndrome
 * whose top bit is set is a single-bit error, and any other non-zero
 * syndrome is an even number of flips - detected and beyond repair. WHICH
 * bit flipped is found by trying all twenty-two, which costs twenty-two
 * evaluations of a small function and needs no syndrome table to be
 * transcribed and checked.
 */
constexpr OtpRow otp_ecc_decode(uint32_t raw24) {
    OtpRow row{};
    row.raw = raw24 & 0x00FFFFFFu;
    uint32_t value = row.raw;
    if ((value & 0x00C00000u) == 0x00C00000u) {
        row.inverted = true;
        value = ~value & 0x00FFFFFFu;
    }
    const uint32_t codeword = value & 0x3FFFFFu;
    row.data = static_cast<uint16_t>(codeword & 0xFFFFu);
    const uint8_t checks = otp_ecc_checks(codeword);
    if (checks == 0u) {
        row.ecc = OtpEcc::ok;
        return row;
    }
    if ((checks & 0x20u) != 0u) {
        // An odd number of flips, which in a code of this distance means
        // one: find it by undoing each of the twenty-two candidates in
        // turn. A search rather than a syndrome table, so that nothing
        // has to be transcribed and checked by eye.
        for (uint8_t bit = 0; bit < 22; ++bit) {
            const uint32_t candidate = codeword ^ (1u << bit);
            if (otp_ecc_checks(candidate) == 0u) {
                row.data = static_cast<uint16_t>(candidate & 0xFFFFu);
                row.ecc = OtpEcc::corrected;
                row.corrected_bit = bit;
                return row;
            }
        }
    }
    row.ecc = OtpEcc::uncorrectable;
    return row;
}

// =============================================================================
// Redundancy without ECC (13.10)
// =============================================================================

/// Best of three, bit by bit: the rule every triple-redundant row of
/// 13.10 is read under.
constexpr uint32_t otp_vote3(uint32_t a, uint32_t b, uint32_t c) {
    return (a & b) | (b & c) | (a & c);
}

/// Three of eight, bit by bit: the CRITICAL flags' own rule (13.4) - a
/// flag counts as set when at least three of its eight copies are.
constexpr uint32_t otp_vote8(const std::array<uint32_t, 8>& rows) {
    uint32_t out = 0;
    for (uint8_t bit = 0; bit < 24; ++bit) {
        uint8_t set = 0;
        for (size_t i = 0; i < rows.size(); ++i) {
            if (((rows[i] >> bit) & 1u) != 0u) {
                ++set;
            }
        }
        if (set >= 3u) {
            out |= 1u << bit;
        }
    }
    return out;
}

// =============================================================================
// The named rows of 13.10, and the flags in them
// =============================================================================

/// The rows this file reads by name. Every number is the device header's
/// own (`hardware/regs/otp_data.h`), so the map is read and never
/// retyped.
struct OtpRowId {
    static constexpr uint16_t chipid0 = OTP_DATA_CHIPID0_ROW;
    static constexpr uint16_t randid0 = OTP_DATA_RANDID0_ROW;
    static constexpr uint16_t rosc_calib = OTP_DATA_ROSC_CALIB_ROW;
    static constexpr uint16_t lposc_calib = OTP_DATA_LPOSC_CALIB_ROW;
    static constexpr uint16_t num_gpios = OTP_DATA_NUM_GPIOS_ROW;
    static constexpr uint16_t info_crc0 = OTP_DATA_INFO_CRC0_ROW;
    static constexpr uint16_t info_crc1 = OTP_DATA_INFO_CRC1_ROW;
    static constexpr uint16_t crit0 = OTP_DATA_CRIT0_ROW;
    static constexpr uint16_t crit1 = OTP_DATA_CRIT1_ROW;
    static constexpr uint16_t boot_flags0 = OTP_DATA_BOOT_FLAGS0_ROW;
    static constexpr uint16_t boot_flags1 = OTP_DATA_BOOT_FLAGS1_ROW;
    static constexpr uint16_t flash_devinfo = OTP_DATA_FLASH_DEVINFO_ROW;
    static constexpr uint16_t usb_boot_flags = OTP_DATA_USB_BOOT_FLAGS_ROW;
};

/// CRIT0, page 0's critical flags: the two that remove an ARCHITECTURE
/// for ever. The factory makes page 0 read-only, so these two cannot be
/// programmed by an owner at all (13.4).
struct OtpCrit0 {
    static constexpr uint32_t arm_disable = OTP_DATA_CRIT0_ARM_DISABLE_BITS;
    static constexpr uint32_t riscv_disable = OTP_DATA_CRIT0_RISCV_DISABLE_BITS;
};

/// CRIT1, page 1's critical flags: secure boot, the two debug disables,
/// the default boot architecture and the glitch detectors. These are the
/// bits this stratum will not write, and the suite prints them as this
/// board's standing record.
struct OtpCrit1 {
    static constexpr uint32_t secure_boot_enable = OTP_DATA_CRIT1_SECURE_BOOT_ENABLE_BITS;
    static constexpr uint32_t secure_debug_disable = OTP_DATA_CRIT1_SECURE_DEBUG_DISABLE_BITS;
    static constexpr uint32_t debug_disable = OTP_DATA_CRIT1_DEBUG_DISABLE_BITS;
    static constexpr uint32_t boot_arch = OTP_DATA_CRIT1_BOOT_ARCH_BITS;
    static constexpr uint32_t glitch_detector_enable =
        OTP_DATA_CRIT1_GLITCH_DETECTOR_ENABLE_BITS;
    static constexpr uint32_t glitch_detector_sens = OTP_DATA_CRIT1_GLITCH_DETECTOR_SENS_BITS;
};

/// The OTP block's OWN copy of the critical flags (13.9's CRITICAL
/// register), latched by the power-up state machine before any code ran.
/// The same flags as CRIT0/CRIT1 above, in different bit positions and
/// already voted by hardware - which is what makes them worth reading
/// beside the rows.
struct OtpCritical {
    static constexpr uint32_t secure_boot_enable = OTP_CRITICAL_SECURE_BOOT_ENABLE_BITS;
    static constexpr uint32_t secure_debug_disable = OTP_CRITICAL_SECURE_DEBUG_DISABLE_BITS;
    static constexpr uint32_t debug_disable = OTP_CRITICAL_DEBUG_DISABLE_BITS;
    static constexpr uint32_t default_archsel = OTP_CRITICAL_DEFAULT_ARCHSEL_BITS;
    static constexpr uint32_t glitch_detector_enable = OTP_CRITICAL_GLITCH_DETECTOR_ENABLE_BITS;
    static constexpr uint32_t glitch_detector_sens = OTP_CRITICAL_GLITCH_DETECTOR_SENS_BITS;
    static constexpr uint32_t arm_disable = OTP_CRITICAL_ARM_DISABLE_BITS;
    static constexpr uint32_t riscv_disable = OTP_CRITICAL_RISCV_DISABLE_BITS;
};

/// The boot paths BOOT_FLAGS0 can close and the configurations it can
/// declare valid (13.10). Named exactly as the register names them, so
/// that a printed flag word can be read against the datasheet.
struct OtpBootFlags0 {
    static constexpr uint32_t disable_sram_window_boot =
        OTP_DATA_BOOT_FLAGS0_DISABLE_SRAM_WINDOW_BOOT_BITS;
    static constexpr uint32_t disable_xip_access_on_sram_entry =
        OTP_DATA_BOOT_FLAGS0_DISABLE_XIP_ACCESS_ON_SRAM_ENTRY_BITS;
    static constexpr uint32_t disable_bootsel_uart_boot =
        OTP_DATA_BOOT_FLAGS0_DISABLE_BOOTSEL_UART_BOOT_BITS;
    static constexpr uint32_t disable_bootsel_usb_picoboot_ifc =
        OTP_DATA_BOOT_FLAGS0_DISABLE_BOOTSEL_USB_PICOBOOT_IFC_BITS;
    static constexpr uint32_t disable_bootsel_usb_msd_ifc =
        OTP_DATA_BOOT_FLAGS0_DISABLE_BOOTSEL_USB_MSD_IFC_BITS;
    static constexpr uint32_t disable_watchdog_scratch =
        OTP_DATA_BOOT_FLAGS0_DISABLE_WATCHDOG_SCRATCH_BITS;
    static constexpr uint32_t disable_power_scratch =
        OTP_DATA_BOOT_FLAGS0_DISABLE_POWER_SCRATCH_BITS;
    static constexpr uint32_t enable_otp_boot = OTP_DATA_BOOT_FLAGS0_ENABLE_OTP_BOOT_BITS;
    static constexpr uint32_t disable_otp_boot = OTP_DATA_BOOT_FLAGS0_DISABLE_OTP_BOOT_BITS;
    static constexpr uint32_t disable_flash_boot = OTP_DATA_BOOT_FLAGS0_DISABLE_FLASH_BOOT_BITS;
    static constexpr uint32_t rollback_required = OTP_DATA_BOOT_FLAGS0_ROLLBACK_REQUIRED_BITS;
    static constexpr uint32_t hashed_partition_table =
        OTP_DATA_BOOT_FLAGS0_HASHED_PARTITION_TABLE_BITS;
    static constexpr uint32_t secure_partition_table =
        OTP_DATA_BOOT_FLAGS0_SECURE_PARTITION_TABLE_BITS;
    static constexpr uint32_t disable_auto_switch_arch =
        OTP_DATA_BOOT_FLAGS0_DISABLE_AUTO_SWITCH_ARCH_BITS;
    static constexpr uint32_t single_flash_binary =
        OTP_DATA_BOOT_FLAGS0_SINGLE_FLASH_BINARY_BITS;
    static constexpr uint32_t override_flash_partition_slot_size =
        OTP_DATA_BOOT_FLAGS0_OVERRIDE_FLASH_PARTITION_SLOT_SIZE_BITS;
    static constexpr uint32_t flash_devinfo_enable =
        OTP_DATA_BOOT_FLAGS0_FLASH_DEVINFO_ENABLE_BITS;
    static constexpr uint32_t fast_sigcheck_rosc_div =
        OTP_DATA_BOOT_FLAGS0_FAST_SIGCHECK_ROSC_DIV_BITS;
    static constexpr uint32_t flash_io_voltage_1v8 =
        OTP_DATA_BOOT_FLAGS0_FLASH_IO_VOLTAGE_1V8_BITS;
    static constexpr uint32_t enable_bootsel_non_default_pll_xosc_cfg =
        OTP_DATA_BOOT_FLAGS0_ENABLE_BOOTSEL_NON_DEFAULT_PLL_XOSC_CFG_BITS;
    static constexpr uint32_t enable_bootsel_led = OTP_DATA_BOOT_FLAGS0_ENABLE_BOOTSEL_LED_BITS;
};

/// Which architecture a core is running (13.9's ARCHSEL_STATUS).
enum class OtpArchitecture : uint8_t { arm = 0, riscv = 1 };

/// The two cores' current architecture, as the register reports it. A
/// core samples ARCHSEL when its warm reset is released, so this is what
/// is running and not what was asked for.
struct OtpArchSel {
    OtpArchitecture core0 = OtpArchitecture::arm;
    OtpArchitecture core1 = OtpArchitecture::arm;
    uint32_t raw = 0;
};

// =============================================================================
// The block
// =============================================================================

/**
 * The array and its control registers, as a monostate resource - READ
 * ONLY, for the reason at the top of this file.
 *
 *   auto row = brio::Otp::read(brio::OtpRowId::num_gpios);
 *   if (row && row->ecc == brio::OtpEcc::ok) { use(row->data); }
 *
 *   const uint64_t id = brio::Otp::chip_id();      // this die's name
 *   const uint32_t crit = brio::Otp::critical();   // what hardware latched
 *
 * The block has NO init: its power-up state machine ran before the first
 * instruction of this image existed (13.3.4), and it is not one of the
 * reset controller's - holding it in reset would reset the processors
 * with it.
 */
struct Otp {
    Otp() = delete;

    static OTP_Type& regs() { return *OTP; }

    /// The one vector (13.3.2), which an app binds as `isr_otp`.
    static constexpr IRQn_Type irq = OTP_IRQ_IRQn;

    /// Which page a row belongs to.
    static constexpr uint8_t page_of(uint16_t row) {
        return static_cast<uint8_t>(row / otp_page_rows);
    }

    // ---- the four windows ----------------------------------------------------

    /// The ECC window: 16 bits, corrected in hardware, all-ones when the
    /// locks refuse the read.
    static uint16_t ecc(uint16_t row) {
        return *reinterpret_cast<volatile uint16_t*>(
            static_cast<uintptr_t>(otp_ecc_window + 2u * row));
    }

    /// The raw window: the row's 24 bits with zeros above them, bypassing
    /// correction. All-ones when the locks refuse the read.
    static uint32_t raw(uint16_t row) {
        return reg_at(otp_raw_window, 4u * row) & 0x00FFFFFFu;
    }

    /// The guarded windows. A refused read, an uncorrectable ECC error or
    /// a failed consistency check FAULTS here instead of returning a
    /// value (13.1.1), so a caller uses these when a wrong answer is
    /// worse than a stopped program - and checks `readable()` first when
    /// it is not.
    static uint16_t ecc_guarded(uint16_t row) {
        return *reinterpret_cast<volatile uint16_t*>(
            static_cast<uintptr_t>(otp_ecc_guarded_window + 2u * row));
    }
    static uint32_t raw_guarded(uint16_t row) {
        return reg_at(otp_raw_guarded_window, 4u * row) & 0x00FFFFFFu;
    }

    /**
     * A row with its ECC verdict, and no fault whatever the locks say:
     * the page's lock is read first, the RAW window second, and 13.6's
     * arithmetic third. Nothing when the row is out of range or the page
     * is inaccessible to this security domain.
     *
     * It reads the raw window on purpose. The ECC window would give the
     * same sixteen bits with less code, and would say nothing at all
     * about whether they needed repairing.
     */
    static std::optional<OtpRow> read(uint16_t row) {
        if (row >= otp_row_count || !readable(page_of(row))) {
            return std::nullopt;
        }
        return otp_ecc_decode(raw(row));
    }

    /// Several rows as one unsigned value, least significant row first -
    /// the shape CHIPID and the calibration pairs are stored in. Nothing
    /// when any row of the run is unreadable.
    static std::optional<uint64_t> read_run(uint16_t first, uint8_t rows) {
        uint64_t value = 0;
        for (uint8_t i = 0; i < rows; ++i) {
            const auto r = read(static_cast<uint16_t>(first + i));
            if (!r) {
                return std::nullopt;
            }
            value |= static_cast<uint64_t>(r->data) << (16 * i);
        }
        return value;
    }

    // ---- the locks, read and decoded (13.5) ----------------------------------

    /// SW_LOCKn for one page. The register file is a plain array of
    /// words at the block's base, so the page number is the index.
    static OtpPageLock page_lock(uint8_t page) {
        if (page >= otp_page_count) {
            return {OtpLock::inaccessible, OtpLock::inaccessible};
        }
        const uint32_t v = reg_at(OTP_BASE, OTP_SW_LOCK0_OFFSET + 4u * page);
        return {
            otp_lock_of((v & OTP_SW_LOCK0_SEC_BITS) >> OTP_SW_LOCK0_SEC_LSB),
            otp_lock_of((v & OTP_SW_LOCK0_NSEC_BITS) >> OTP_SW_LOCK0_NSEC_LSB),
        };
    }

    /// Whether this program may read the page. Every brio program on this
    /// chip runs Secure (the bootrom hands over that way), so the Secure
    /// field is the one that decides.
    static bool readable(uint8_t page) {
        return page_lock(page).secure != OtpLock::inaccessible;
    }

    // ---- what the chip says about itself (13.10) -----------------------------

    /// CHIPID0..3: a 64-bit random public identifier written at
    /// manufacturing test - the nearest thing this die has to a serial
    /// number, and what the bootrom's `get_sys_info` reports as the
    /// device id.
    static std::optional<uint64_t> chip_id() { return read_run(OtpRowId::chipid0, 4); }

    /// RANDID0..7: 128 bits of per-device randomness, also written at
    /// test. 13.10 calls it private and then says it is not meaningfully
    /// private while the USB bootloader's OTP access point is enabled -
    /// so it is a seed, not a secret.
    static std::optional<std::array<uint16_t, 8>> random_id() {
        std::array<uint16_t, 8> out{};
        for (uint8_t i = 0; i < 8; ++i) {
            const auto r = read(static_cast<uint16_t>(OtpRowId::randid0 + i));
            if (!r) {
                return std::nullopt;
            }
            out[i] = r->data;
        }
        return out;
    }

    /// ROSC_CALIB: the ring oscillator's frequency in kHz, measured at
    /// 1.1 V and room temperature with its control registers at reset.
    static std::optional<uint16_t> rosc_calib_khz() {
        const auto r = read(OtpRowId::rosc_calib);
        return r ? std::optional<uint16_t>{r->data} : std::nullopt;
    }
    /// LPOSC_CALIB: the low-power oscillator's frequency in Hz, likewise.
    static std::optional<uint16_t> lposc_calib_hz() {
        const auto r = read(OtpRowId::lposc_calib);
        return r ? std::optional<uint16_t>{r->data} : std::nullopt;
    }

    /// NUM_GPIOS: how many bank 0 pads this package bonds - 48 on the
    /// QFN-80 and 30 on the QFN-60. The same fact SYSINFO's PACKAGE_SEL
    /// carries, written by the factory into the array.
    static std::optional<uint8_t> num_gpios() {
        const auto r = read(OtpRowId::num_gpios);
        return r ? std::optional<uint8_t>{static_cast<uint8_t>(r->data & 0xFFu)} : std::nullopt;
    }

    /// INFO_CRC0/1: a CRC-32 over rows 0x00..0x6b, in the reflected
    /// variant 13.10 names (polynomial 0x04c11db7, input and output
    /// reflected, all-ones seed, all-ones final XOR - which is the CRC-32
    /// of zlib and NOT the MPEG-2 form util/crc.hpp carries). The value
    /// is reported; checking it would mean reading a hundred rows and a
    /// second CRC variant, which nothing needs yet.
    static std::optional<uint32_t> info_crc() {
        const auto lo = read(OtpRowId::info_crc0);
        const auto hi = read(OtpRowId::info_crc1);
        if (!lo || !hi) {
            return std::nullopt;
        }
        return static_cast<uint32_t>(lo->data) | (static_cast<uint32_t>(hi->data) << 16);
    }

    /// FLASH_DEVINFO: what the factory or the owner recorded about the
    /// external memories, valid only when BOOT_FLAGS0's
    /// FLASH_DEVINFO_ENABLE is set. Reported raw: decoding the two size
    /// fields belongs to the flash chapter, which is the only thing that
    /// can check the answer against a chip.
    static std::optional<uint16_t> flash_devinfo() {
        const auto r = read(OtpRowId::flash_devinfo);
        return r ? std::optional<uint16_t>{r->data} : std::nullopt;
    }

    /// CRIT0 and CRIT1 as the hardware reads them: eight redundant rows
    /// each, three-of-eight voted (13.4). These are the flags the hard
    /// rules of this project forbid a program to set; reading them is how
    /// a board's standing state is recorded.
    static std::optional<uint32_t> crit0() { return voted8(OtpRowId::crit0); }
    static std::optional<uint32_t> crit1() { return voted8(OtpRowId::crit1); }

    /// BOOT_FLAGS0/1 and USB_BOOT_FLAGS: three redundant rows each,
    /// majority voted.
    static std::optional<uint32_t> boot_flags0() { return voted3(OtpRowId::boot_flags0); }
    static std::optional<uint32_t> boot_flags1() { return voted3(OtpRowId::boot_flags1); }
    static std::optional<uint32_t> usb_boot_flags() { return voted3(OtpRowId::usb_boot_flags); }

    // ---- the block's own registers (13.9) ------------------------------------

    /// CRITICAL: the critical flags as the power-up state machine latched
    /// them, before any code ran. Hardware has already done the vote, so
    /// this and `crit0()`/`crit1()` should agree - in different bit
    /// positions, which is why both are offered.
    static uint32_t critical() { return regs().CRITICAL; }

    /// KEY_VALID: which of the eight hardware access keys were enrolled
    /// at boot. A one means the key exists and is therefore unreadable.
    static uint8_t key_valid() { return static_cast<uint8_t>(regs().KEY_VALID & 0xFFu); }

    /// DEBUGEN and DEBUGEN_LOCK: which debug features have been re-enabled
    /// after a critical flag or a key disabled them, and whether that
    /// re-enabling has been locked out. Read only here - writing them is
    /// the debug chapter's business and a way to lose a board.
    static uint32_t debugen() { return regs().DEBUGEN; }
    static uint32_t debugen_lock() { return regs().DEBUGEN_LOCK; }

    /// ARCHSEL_STATUS (offset 0x15c): which architecture each core is
    /// RUNNING. Updated when a core's warm reset is released.
    static OtpArchSel archsel_status() {
        const uint32_t v = regs().ARCHSEL_STATUS & OTP_ARCHSEL_STATUS_BITS;
        return {
            (v & OTP_ARCHSEL_STATUS_CORE0_BITS) != 0u ? OtpArchitecture::riscv
                                                      : OtpArchitecture::arm,
            (v & OTP_ARCHSEL_STATUS_CORE1_BITS) != 0u ? OtpArchitecture::riscv
                                                      : OtpArchitecture::arm,
            v,
        };
    }

    /// ARCHSEL, the request beside the status: which architecture each
    /// core will be at its next warm reset. READ ONLY here - switching a
    /// core's architecture is the multicore chapter's verb, and on this
    /// stratum the architecture is a property of the image.
    static uint32_t archsel() { return regs().ARCHSEL & OTP_ARCHSEL_BITS; }

    /// BOOTDIS: whether the bootrom will ignore the scratch-register boot
    /// vectors next time. Read; the two bits are set by a boot stage that
    /// has soft-locked pages, which this stratum does not do.
    static uint32_t bootdis() { return regs().BOOTDIS & OTP_BOOTDIS_BITS; }

    /// USR.DCTRL: whether the data windows are reachable at all. It is
    /// set at reset and clear only while the programming bridge owns the
    /// array, so a false here means something else is programming OTP -
    /// and every read above would take a bus error.
    static bool data_window_enabled() { return (regs().USR & OTP_USR_DCTRL_BITS) != 0u; }

    /// DBG.CUSTOMER_RMA_FLAG: the decommissioning flag of 13.7. When it
    /// is set the factory test port is back and pages 3..61 are
    /// inaccessible.
    static bool rma_flag() { return (regs().DBG & OTP_DBG_CUSTOMER_RMA_FLAG_BITS) != 0u; }

    /// DBG whole: the power-on state machine's own diagnosis, for a
    /// program that wants to report it.
    static uint32_t debug_status() { return regs().DBG; }

    /// INTR / INTS: the five sources of 13.3.2 - a Secure read refused, a
    /// Non-secure read refused, a write refused, the programming bridge's
    /// completion flag, and a data-port access made while the bridge owns
    /// the array. Raw and masked; INTE is left alone because nothing here
    /// arms an interrupt.
    static uint32_t interrupt_raw() { return regs().INTR; }
    static uint32_t interrupt_status() { return regs().INTS; }
    /// The three refusal flags are write-one-to-clear; the other two
    /// follow their sources.
    static void clear_interrupts(uint32_t flags) { regs().INTR = flags; }

private:
    /// A three-of-eight voted flag word: the row and its seven copies.
    static std::optional<uint32_t> voted8(uint16_t first) {
        std::array<uint32_t, 8> rows{};
        for (uint8_t i = 0; i < 8; ++i) {
            const uint16_t r = static_cast<uint16_t>(first + i);
            if (r >= otp_row_count || !readable(page_of(r))) {
                return std::nullopt;
            }
            rows[i] = raw(r);
        }
        return otp_vote8(rows);
    }

    /// A majority-of-three voted flag word.
    static std::optional<uint32_t> voted3(uint16_t first) {
        for (uint8_t i = 0; i < 3; ++i) {
            const uint16_t r = static_cast<uint16_t>(first + i);
            if (r >= otp_row_count || !readable(page_of(r))) {
                return std::nullopt;
            }
        }
        return otp_vote3(raw(first), raw(static_cast<uint16_t>(first + 1)),
                         raw(static_cast<uint16_t>(first + 2)));
    }
};

} // namespace brio
