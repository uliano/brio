/*
 * sysinfo.hpp
 *
 * SYSINFO (datasheet 12.15.1): what the chip says about itself - which
 * part and which stepping this is, which package the die is in, whether
 * it is silicon at all - for a program that reports it in a banner or
 * keys a workaround to an erratum. The smallest use, in the shape the
 * apps use it:
 *
 *     if (brio::ChipId::stepping() == brio::Stepping::a2) { ... RP2350-E9 is live ... }
 *     const brio::Package pkg = brio::ChipId::package_sel();
 *
 * Which context calls what. Every verb is a register read after a
 * release of its block from reset (resets.hpp: one alias write and a
 * bounded wait for RESET_DONE, a confirmation once the block is out);
 * nothing masks interrupts and nothing keeps storage - read() returns
 * its answer by value. Any context that may spend that bounded wait may
 * call them; the apps call them from main().
 *
 * Three facts, and one of them is load-bearing for every later chapter:
 *
 *  - CHIP_ID, the JEDEC JEP-106 identifier: a manufacturer, a part
 *    number and the silicon revision. The revision alone does not name
 *    the stepping the errata are keyed by: appendix C gives A2 the value
 *    0x2 and BOTH A3 and A4 the value 0x3, the two told apart by the
 *    bootrom's version byte at 0x13 (3 on A3, 4 on A4; bootrom.hpp).
 *    So the raw field is what read() offers, and stepping() is the
 *    answer to "which stepping", from the two together.
 *  - PACKAGE_SEL, which says which package the die is in: 0 = QFN-80
 *    (48 GPIO, eight ADC inputs), 1 = QFN-60 (30 and four). Since the
 *    die is one, every pin's registers exist either way and only the
 *    pad is absent - so this bit is how a program that was not told at
 *    build time finds out (device.hpp's `package` is the other half of
 *    that story).
 *  - "am I real" - silicon (ASIC), an FPGA or a simulation - which a
 *    suite can then state rather than assume. The answer is not this
 *    block's: the chip has two registers called PLATFORM, and SYSINFO's
 *    (12.15.1, offset 0x08) is the pre-production indicator - it reads
 *    all-zero on a production part, ASIC bit included (measured on
 *    stepping A2). The one that answers is the testbench manager's,
 *    TBMAN PLATFORM, whose ASIC bit resets to 1 and reads 1 on the same
 *    part; asic() reads that one, and releases that block.
 *
 * GITREF_RP2350 is the git hash of the chip's source, for the record.
 * There is no per-die serial number here: a board's identity is its
 * flash chip's unique id, which is the flash chapter's.
 *
 * The block is one of the reset controller's (7.5): read() releases it
 * first, so the first suite banner of a program costs one release.
 */

#pragma once

#include <stdint.h>

#include "rp2350/bootrom.hpp"
#include "rp2350/device.hpp"
#include "rp2350/resets.hpp"

namespace brio {

/// The steppings appendix C names; `unknown` for any other pair of a
/// revision and a bootrom version, which a program states rather than
/// guesses at.
enum class Stepping : uint8_t { a2, a3, a4, unknown };

/// The stepping's name as the datasheet writes it, for a banner.
constexpr const char* stepping_name(Stepping s) {
    return s == Stepping::a2 ? "A2" : s == Stepping::a3 ? "A3" : s == Stepping::a4 ? "A4" : "unknown";
}

struct ChipId {
    uint16_t manufacturer;   ///< JEP-106 manufacturer, 0x493 for Raspberry Pi
    uint16_t part;           ///< 0x0004 = RP2350
    uint8_t revision;        ///< the silicon revision, raw (see the constants)

    /// The manufacturer field of this chip is eleven bits with the
    /// JEP-106 stop bit split off beside it, so it reads 0x493 - the
    /// same bits the RP2040 reports as 0x927 with the stop bit included,
    /// which is why the two chips' constants differ by a shift and a
    /// bit and not by a manufacturer.
    static constexpr uint16_t raspberry_pi = 0x493;
    static constexpr uint16_t rp2350 = 0x0004;

    /// The revision values appendix C names: 0x2 on A2, 0x3 on A3 and
    /// on A4 alike (stepping() tells those two apart).
    static constexpr uint8_t revision_a2 = 0x2;
    static constexpr uint8_t revision_a3_a4 = 0x3;

    /// Read CHIP_ID (releasing SYSINFO from reset if it still is).
    static ChipId read() {
        (void)Resets::release(ResetBlock::sysinfo);
        const uint32_t id = SYSINFO->CHIP_ID;
        return {
            .manufacturer = static_cast<uint16_t>((id & SYSINFO_CHIP_ID_MANUFACTURER_BITS) >>
                                                  SYSINFO_CHIP_ID_MANUFACTURER_LSB),
            .part = static_cast<uint16_t>((id & SYSINFO_CHIP_ID_PART_BITS) >>
                                          SYSINFO_CHIP_ID_PART_LSB),
            .revision = static_cast<uint8_t>((id & SYSINFO_CHIP_ID_REVISION_BITS) >>
                                             SYSINFO_CHIP_ID_REVISION_LSB),
        };
    }

    /// Which stepping this die is (appendix C): CHIP_ID's revision, and
    /// on revision 0x3 the bootrom's version byte, 3 for A3 and 4 for A4.
    /// Two reads after SYSINFO's release; any context read() may run in.
    static Stepping stepping() {
        const uint8_t revision = read().revision;
        if (revision == revision_a2) {
            return Stepping::a2;
        }
        if (revision == revision_a3_a4) {
            const uint8_t rom = Bootrom::version();
            return rom == 3u ? Stepping::a3 : rom == 4u ? Stepping::a4 : Stepping::unknown;
        }
        return Stepping::unknown;
    }

    /// Which package the die is in, as the chip reports it. What a
    /// program built without BRIO_RP2350_PACKAGE_PINS asks before
    /// touching a pad above GP29 (rp2350/pin.hpp does exactly that).
    static Package package_sel() {
        (void)Resets::release(ResetBlock::sysinfo);
        return (SYSINFO->PACKAGE_SEL & SYSINFO_PACKAGE_SEL_BITS) != 0u ? Package::qfn60
                                                                       : Package::qfn80;
    }

    /// True on real silicon (as against an FPGA or a simulation): the
    /// testbench manager's PLATFORM.ASIC, never SYSINFO's register of the
    /// same name (the header comment says why).
    static bool asic() {
        (void)Resets::release(ResetBlock::tbman);
        return (TBMAN->PLATFORM & TBMAN_PLATFORM_ASIC_BITS) != 0u;
    }

    /// The chip's source git hash, for the record.
    static uint32_t gitref() {
        (void)Resets::release(ResetBlock::sysinfo);
        return SYSINFO->GITREF_RP2350;
    }
};

} // namespace brio
