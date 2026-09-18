/*
 * sysinfo.hpp
 *
 * SYSINFO (datasheet 12.15.1): what the chip says about itself. Three
 * facts, and one of them is load-bearing for every later chapter:
 *
 *  - CHIP_ID, the JEDEC JEP-106 identifier: a manufacturer, a part
 *    number and the silicon REVISION the errata are keyed by. The
 *    datasheet names three steppings and the value each reports (A2 =
 *    0x2, A3 = 0x3, A4 = 0x8 - appendix C), and the numbering is not
 *    contiguous, so the raw field is what this file offers and the three
 *    documented values are constants beside it.
 *  - PACKAGE_SEL, which says WHICH PACKAGE the die is in: 0 = QFN-80
 *    (48 GPIO, eight ADC inputs), 1 = QFN-60 (30 and four). Since the
 *    die is one, every pin's registers exist either way and only the
 *    PAD is absent - so this bit is how a program that was not told at
 *    build time finds out (device.hpp's `package` is the other half of
 *    that story).
 *  - "am I real" - silicon (ASIC), an FPGA or a simulation - which a
 *    suite can then state rather than assume. THE ANSWER IS NOT THIS
 *    BLOCK'S: the chip has TWO registers called PLATFORM, and SYSINFO's
 *    (12.15.1, offset 0x08) is the PRE-PRODUCTION indicator - it reads
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

#include "rp2350/device.hpp"
#include "rp2350/resets.hpp"

namespace brio {

struct ChipId {
    uint16_t manufacturer;   ///< JEP-106 manufacturer, 0x493 for Raspberry Pi
    uint16_t part;           ///< 0x0004 = RP2350
    uint8_t revision;        ///< the silicon revision, raw (see the constants)

    /// The manufacturer field of THIS chip is eleven bits with the
    /// JEP-106 stop bit split off beside it, so it reads 0x493 - the
    /// same bits the RP2040 reports as 0x927 with the stop bit included,
    /// which is why the two chips' constants differ by a shift and a
    /// bit and not by a manufacturer.
    static constexpr uint16_t raspberry_pi = 0x493;
    static constexpr uint16_t rp2350 = 0x0004;

    /// The steppings appendix C names, by the value CHIP_ID reports.
    static constexpr uint8_t revision_a2 = 0x2;
    static constexpr uint8_t revision_a3 = 0x3;
    static constexpr uint8_t revision_a4 = 0x8;

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
