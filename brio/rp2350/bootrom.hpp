/*
 * bootrom.hpp
 *
 * THE MASK ROM'S PUBLIC FUNCTIONS (datasheet 5.4): 32 kB of code that
 * booted this image and is still there at address zero, with a table
 * that says where its entry points are. The RP2040 had one of these; the
 * RP2350's is larger, its table has an entry PER ARCHITECTURE, and both
 * of those facts are why this file exists rather than a handful of
 * constants in whoever needs one.
 *
 * WHY THE TABLE AND NOT A FIXED ADDRESS. 5.4.1: the locations of the
 * functions "may change with each bootrom release", so what is fixed is
 * a handful of well-known WORDS at the bottom of the ROM - a magic
 * number, a version byte, and pointers to the table and to two lookup
 * helpers that walk it. A caller asks for a function by a two-character
 * CODE ('G','S' for get_sys_info) and gets an address back, or nothing.
 *
 * THE TWO ARCHITECTURES ARE TWO LOOKUPS, NOT ONE (5.4.1's tables 453 and
 * 454). Arm code finds the helpers at 0x14..0x18; RISC-V code finds its
 * own at 0x7df6..0x7dfc, near the top of the ROM. And the two ask for
 * different things: on Arm the table holds a POINTER to the function, so
 * the helper that returns the stored VALUE is the right one; on RISC-V
 * the table entry IS a jump instruction, so the helper that returns the
 * ENTRY'S OWN ADDRESS is. This file asks `core_kind` - a constant, not
 * the preprocessor - and does the right one.
 *
 * SECURITY STATE. brio runs Secure on the Arm half (the bootrom hands
 * over that way and nothing here writes the SAU), so functions are
 * looked up with the Secure Arm flag; the Non-secure entry points, which
 * must be enabled one by one by Secure code before they exist at all,
 * are not offered. On RISC-V there is one entry point per function and
 * no flag to choose.
 *
 * WHAT IS WRAPPED, AND WHAT DELIBERATELY IS NOT. Three read-only things:
 * `get_sys_info`, `get_partition_table_info` and the ROM's own git
 * revision word. Not here:
 *
 *   - THE FLASH FUNCTIONS. They disconnect the QSPI interface from the
 *     execute-in-place window, so they must run from RAM with interrupts
 *     masked; that discipline is a chapter of its own and belongs beside
 *     the flash driver, not in a lookup table's file.
 *   - `reboot()`. This stratum's reboot is the watchdog's (reset.hpp),
 *     which is what the reset chapter measured; a second spelling of the
 *     same act would be a second thing to keep true.
 *   - `otp_access()`. It is the bootrom's OTP PROGRAMMING entry point -
 *     its `row_and_flags` argument carries an IS_WRITE bit - and this
 *     project's rule is that no write path to OTP exists in the tree.
 *     The read side of OTP is memory-mapped and needs no ROM call
 *     (rp2350/otp.hpp), so there is nothing lost and one irreversible
 *     mistake made impossible.
 *
 * THE BOOT LOCKS (5.4.4) ARE NOT CLAIMED. The ROM checks them only when
 * boot lock 7 has been claimed to turn checking on, and nothing in brio
 * claims it; the two functions wrapped here own no hardware, so there is
 * nothing to arbitrate. A program that starts using the ROM's flash or
 * OTP entry points is the program that will need the locks, and it can
 * have them then.
 *
 * NOTHING HERE IS CALLED FROM AN INTERRUPT. These are ordinary C
 * functions in ROM with ordinary stack needs, and 5.4.8's own notes warn
 * that some want a good deal of stack on RISC-V.
 */

#pragma once

#include <stdint.h>

#include <array>
#include <optional>

#include "rp2350/core.hpp"
#include "rp2350/device.hpp"

namespace brio {

// =============================================================================
// The well-known words (5.4.1)
// =============================================================================

/// Where the fixed words live. The Arm set is at the bottom of the ROM
/// beside the boot vector table; the RISC-V set is just below the ROM's
/// RISC-V entry point at 0x7dfc.
struct BootromAddress {
    static constexpr uint32_t magic = 0x00000010u;          ///< 'M', 'u', 0x02
    static constexpr uint32_t version = 0x00000013u;        ///< one byte
    static constexpr uint32_t table_arm = 0x00000014u;      ///< 16-bit pointer
    static constexpr uint32_t lookup_value_arm = 0x00000016u;
    static constexpr uint32_t lookup_entry_arm = 0x00000018u;
    static constexpr uint32_t table_riscv = 0x00007df6u;
    static constexpr uint32_t lookup_value_riscv = 0x00007df8u;
    static constexpr uint32_t lookup_entry_riscv = 0x00007dfau;
    static constexpr uint32_t entry_riscv = 0x00007dfcu;
};

/// The three magic bytes at 0x10, as one 24-bit value read little-endian:
/// 'M', 'u', 0x02.
inline constexpr uint32_t bootrom_magic = 0x02u << 16 | static_cast<uint32_t>('u') << 8 |
                                          static_cast<uint32_t>('M');

/**
 * The flags the lookup helpers take (5.4.1). The datasheet names them
 * and the ROM's own ABI fixes their values, which the SDK publishes in
 * `boot/bootrom_constants.h`; they are stated here because a lookup
 * cannot be made without them and because a wrong one silently returns
 * nothing rather than misbehaving.
 */
struct BootromFlag {
    static constexpr uint32_t func_riscv = 0x0001u;
    static constexpr uint32_t func_riscv_far = 0x0003u;
    static constexpr uint32_t func_arm_secure = 0x0004u;
    static constexpr uint32_t func_arm_non_secure = 0x0010u;
    static constexpr uint32_t data = 0x0040u;
};

/// The error codes of 5.4.3, as the negative values the ROM returns.
struct BootromError {
    static constexpr int32_t ok = 0;
    static constexpr int32_t not_permitted = -4;
    static constexpr int32_t invalid_arg = -5;
    static constexpr int32_t invalid_address = -10;
    static constexpr int32_t bad_alignment = -11;
    static constexpr int32_t invalid_state = -12;
    static constexpr int32_t buffer_too_small = -13;
    static constexpr int32_t precondition_not_met = -14;
    static constexpr int32_t modified_data = -15;
    static constexpr int32_t invalid_data = -16;
    static constexpr int32_t not_found = -17;
    static constexpr int32_t unsupported_modification = -18;
    static constexpr int32_t lock_required = -19;
};

/// The two-character codes of 5.4.7, as `rom_table_code()` computes them.
constexpr uint32_t bootrom_code(char c1, char c2) {
    return static_cast<uint32_t>(static_cast<uint8_t>(c1)) |
           (static_cast<uint32_t>(static_cast<uint8_t>(c2)) << 8);
}

/// The codes this file uses, by name.
struct BootromCode {
    static constexpr uint32_t get_sys_info = bootrom_code('G', 'S');
    static constexpr uint32_t get_partition_table_info = bootrom_code('G', 'P');
    static constexpr uint32_t git_revision = bootrom_code('G', 'R');
    static constexpr uint32_t partition_table_ptr = bootrom_code('P', 'T');
};

/// `get_sys_info`'s flags (5.4.8.17). The first word of the answer is
/// the subset of these the ROM actually served, which a caller must
/// check before reading any of the rest.
struct SysInfoFlag {
    static constexpr uint32_t chip_info = 0x0001u;       ///< 3 words: package, id low, id high
    static constexpr uint32_t critical = 0x0002u;        ///< 1 word: the OTP CRITICAL register
    static constexpr uint32_t cpu_info = 0x0004u;        ///< 1 word: 0 Arm, 1 RISC-V
    static constexpr uint32_t flash_dev_info = 0x0008u;  ///< 1 word: OTP FLASH_DEVINFO's form
    static constexpr uint32_t boot_random = 0x0010u;     ///< 4 words: the per-boot random number
    static constexpr uint32_t nonce = 0x0020u;           ///< not supported on this silicon
    static constexpr uint32_t boot_info = 0x0040u;       ///< 4 words: how this boot went
    /// Everything this file knows how to decode.
    static constexpr uint32_t all =
        chip_info | critical | cpu_info | flash_dev_info | boot_random | boot_info;
};

/// `get_partition_table_info`'s flags (5.4.8.16).
struct PartitionInfoFlag {
    static constexpr uint32_t pt_info = 0x0001u;
    static constexpr uint32_t location_and_flags = 0x0010u;
    static constexpr uint32_t id = 0x0020u;
    static constexpr uint32_t family_ids = 0x0040u;
    static constexpr uint32_t name = 0x0080u;
    static constexpr uint32_t single_partition = 0x8000u;
};

/// What `get_sys_info` said, sorted out: each field carries whether the
/// ROM served it, because the flags a caller ASKS for and the flags the
/// ROM supports are not the same set.
struct BootromSysInfo {
    uint32_t served = 0;            ///< the first word: what is actually below
    uint32_t package_sel = 0;       ///< CHIP_INFO word 0
    uint64_t device_id = 0;         ///< CHIP_INFO words 1 and 2: the OTP CHIPID
    uint32_t critical = 0;          ///< CRITICAL word 0
    uint32_t cpu = 0;               ///< CPU_INFO word 0: 0 Arm, 1 RISC-V
    uint32_t flash_dev_info = 0;    ///< FLASH_DEV_INFO word 0
    std::array<uint32_t, 4> boot_random{};  ///< BOOT_RANDOM, 128 bits made at boot
    std::array<uint32_t, 4> boot_info{};    ///< BOOT_INFO, the boot's own diagnosis

    constexpr bool has(uint32_t flag) const { return (served & flag) != 0u; }
};

// =============================================================================
// The ROM
// =============================================================================

/**
 * The bootrom as a monostate.
 *
 *   if (brio::Bootrom::present()) {
 *       uint32_t out[8];
 *       const int32_t n = brio::Bootrom::get_sys_info(out, 8,
 *                              brio::SysInfoFlag::chip_info);
 *   }
 *
 * `lookup_function()` and `lookup_data()` are the general verbs; the
 * three below them are the wrappers this stratum needs. Every one of
 * them answers with nothing rather than jumping to address zero when the
 * table is not where it should be.
 */
struct Bootrom {
    Bootrom() = delete;

    /// The lookup helpers' signature (5.4.1): a code and a flag word in,
    /// an address out.
    using Lookup = void* (*)(uint32_t code, uint32_t flags);

    /// The two functions this file wraps, as the ROM declares them.
    using SysInfoFn = int32_t (*)(uint32_t* out, uint32_t out_words, uint32_t flags);
    using PartitionInfoFn = int32_t (*)(uint32_t* out, uint32_t out_words,
                                        uint32_t flags_and_partition);

    /// Which flag a FUNCTION lookup carries on this half: the Secure Arm
    /// entry point, or the RISC-V one.
    static constexpr uint32_t function_flag =
        core_kind == CoreKind::hazard3 ? BootromFlag::func_riscv : BootromFlag::func_arm_secure;

    /// True when the three bytes at 0x10 are 'M', 'u', 2 - which 5.4.1
    /// says is what makes the other fixed words trustworthy.
    static bool present() { return (word_at(BootromAddress::magic) & 0x00FFFFFFu) == bootrom_magic; }

    /// The ROM's version byte (5.4.1): 2 on A2 silicon. Informational,
    /// and explicitly NOT a way to infer where anything lives.
    static uint8_t version() {
        return *reinterpret_cast<const volatile uint8_t*>(rom_address(BootromAddress::version));
    }

    /// Where the entry table starts. Read for the record; the helpers
    /// below are what walks it.
    static const void* table() {
        return reinterpret_cast<const void*>(static_cast<uintptr_t>(
            pointer_at(core_kind == CoreKind::hazard3 ? BootromAddress::table_riscv
                                                      : BootromAddress::table_arm)));
    }

    /// The helper that returns the VALUE stored in a table entry: what
    /// an Arm function lookup and a data lookup on either half want.
    static Lookup lookup_value() {
        return reinterpret_cast<Lookup>(static_cast<uintptr_t>(
            pointer_at(core_kind == CoreKind::hazard3 ? BootromAddress::lookup_value_riscv
                                                      : BootromAddress::lookup_value_arm)));
    }

    /// The helper that returns the ADDRESS OF the table entry: what a
    /// RISC-V function lookup wants, the entry itself being a jump.
    static Lookup lookup_entry() {
        return reinterpret_cast<Lookup>(static_cast<uintptr_t>(
            pointer_at(core_kind == CoreKind::hazard3 ? BootromAddress::lookup_entry_riscv
                                                      : BootromAddress::lookup_entry_arm)));
    }

    /// One public function by its code, or nullptr. The architecture
    /// decides both which helper is asked and which flag it is asked
    /// with - see the file header.
    static void* lookup_function(uint32_t code) {
        if (!present()) {
            return nullptr;
        }
        const Lookup helper =
            core_kind == CoreKind::hazard3 ? lookup_entry() : lookup_value();
        return helper == nullptr ? nullptr : helper(code, function_flag);
    }

    /// One ROM DATA location by its code, or nullptr. Data is a stored
    /// pointer on both halves, so the value helper serves both.
    static void* lookup_data(uint32_t code) {
        if (!present()) {
            return nullptr;
        }
        const Lookup helper = lookup_value();
        return helper == nullptr ? nullptr : helper(code, BootromFlag::data);
    }

    // ---- the wrappers --------------------------------------------------------

    /// The ROM's own git revision (5.4.8.19): the eight most significant
    /// hex digits of the revision this mask ROM was built from. Nothing
    /// when the lookup failed.
    static std::optional<uint32_t> git_revision() {
        const void* p = lookup_data(BootromCode::git_revision);
        if (p == nullptr) {
            return std::nullopt;
        }
        return *static_cast<const volatile uint32_t*>(p);
    }

    /**
     * `get_sys_info` (5.4.8.17), raw: fills `out` and returns the number
     * of words filled, or one of `BootromError`'s negative codes.
     * `BootromError::not_found` when the function is not in the table at
     * all, which is this wrapper's own answer and not the ROM's.
     */
    static int32_t get_sys_info(uint32_t* out, uint32_t out_words, uint32_t flags) {
        const auto fn = reinterpret_cast<SysInfoFn>(lookup_function(BootromCode::get_sys_info));
        return fn == nullptr ? BootromError::not_found : fn(out, out_words, flags);
    }

    /**
     * `get_sys_info` decoded: the flags asked for, the answer's own
     * "what I served" word checked, and the words that follow taken in
     * the ORDER the flags are listed in - which is the whole protocol,
     * there being no tags in the buffer.
     *
     * Nothing when the call failed. A call that succeeds but serves less
     * than was asked for is NOT a failure: `served` says what is there.
     */
    static std::optional<BootromSysInfo> sys_info(uint32_t flags = SysInfoFlag::all) {
        std::array<uint32_t, 16> buffer{};
        const int32_t n = get_sys_info(buffer.data(), static_cast<uint32_t>(buffer.size()), flags);
        if (n < 1) {
            return std::nullopt;
        }
        BootromSysInfo info{};
        info.served = buffer[0];
        uint32_t i = 1;
        const uint32_t words = static_cast<uint32_t>(n);
        const auto take = [&](uint32_t count) -> const uint32_t* {
            if (i + count > words) {
                return nullptr;
            }
            const uint32_t* p = buffer.data() + i;
            i += count;
            return p;
        };
        if (info.has(SysInfoFlag::chip_info)) {
            if (const uint32_t* p = take(3)) {
                info.package_sel = p[0];
                info.device_id = static_cast<uint64_t>(p[1]) |
                                 (static_cast<uint64_t>(p[2]) << 32);
            }
        }
        if (info.has(SysInfoFlag::critical)) {
            if (const uint32_t* p = take(1)) {
                info.critical = p[0];
            }
        }
        if (info.has(SysInfoFlag::cpu_info)) {
            if (const uint32_t* p = take(1)) {
                info.cpu = p[0];
            }
        }
        if (info.has(SysInfoFlag::flash_dev_info)) {
            if (const uint32_t* p = take(1)) {
                info.flash_dev_info = p[0];
            }
        }
        if (info.has(SysInfoFlag::boot_random)) {
            if (const uint32_t* p = take(4)) {
                for (uint32_t k = 0; k < 4; ++k) {
                    info.boot_random[k] = p[k];
                }
            }
        }
        if (info.has(SysInfoFlag::boot_info)) {
            if (const uint32_t* p = take(4)) {
                for (uint32_t k = 0; k < 4; ++k) {
                    info.boot_info[k] = p[k];
                }
            }
        }
        return info;
    }

    /**
     * `get_partition_table_info` (5.4.8.16), raw. It reports
     * `BootromError::precondition_not_met` when no partition table has
     * been loaded - which is the ordinary answer on a board whose image
     * carries none, and the reason this wrapper is offered without
     * `load_partition_table()` beside it: loading one wants a three
     * kilobyte work area and a program that has a partition table to
     * load.
     */
    static int32_t get_partition_table_info(uint32_t* out, uint32_t out_words,
                                            uint32_t flags_and_partition) {
        const auto fn = reinterpret_cast<PartitionInfoFn>(
            lookup_function(BootromCode::get_partition_table_info));
        return fn == nullptr ? BootromError::not_found : fn(out, out_words, flags_and_partition);
    }

private:
    /// A ROM address the compiler cannot fold. The fixed words live in
    /// the first page of the address space, which gcc takes for the null
    /// page and refuses to read through a constant - the same trap the
    /// RP2040's flash driver met at the same addresses.
    static uintptr_t rom_address(uint32_t address) {
        uintptr_t a = address;
        asm volatile("" : "+r"(a));
        return a;
    }
    /// A 32-bit read from a fixed ROM address.
    static uint32_t word_at(uint32_t address) {
        return *reinterpret_cast<const volatile uint32_t*>(rom_address(address));
    }
    /// A 16-bit pointer cell: the ROM is 32 kB, so every address in it
    /// fits in a halfword and the fixed table stores them that way.
    static uint32_t pointer_at(uint32_t address) {
        return *reinterpret_cast<const volatile uint16_t*>(rom_address(address));
    }
};

} // namespace brio
