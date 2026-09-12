/*
 * sysinfo.hpp
 *
 * SYSINFO (datasheet 2.20): what the chip says about itself. CHIP_ID is
 * the JEDEC JEP-106 identifier - manufacturer 0x927 (Raspberry Pi),
 * part 0x0002 (the RP2040) and the silicon REVISION the errata are keyed
 * by (1 = B1, 2 = B2, the one on every board sold since 2021; 0 = B0) -
 * and GITREF_RP2040 the git hash of the chip's source. There is no
 * per-die serial number on this chip: a board's identity is its flash
 * chip's unique id, read over the SSI, which is a later chapter.
 *
 * The block is one of the reset controller's (2.14): read() releases it
 * first, so the first suite banner of a program costs one release.
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"
#include "rp2040/resets.hpp"

namespace brio {

struct ChipId {
    uint16_t manufacturer;   ///< JEP-106 manufacturer, 0x927 for Raspberry Pi
    uint16_t part;           ///< 0x0002 = RP2040
    uint8_t revision;        ///< the silicon revision: 0 = B0, 1 = B1, 2 = B2

    static constexpr uint16_t raspberry_pi = 0x927;
    static constexpr uint16_t rp2040 = 0x0002;

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

    /// The chip's source git hash, for the record.
    static uint32_t gitref() {
        (void)Resets::release(ResetBlock::sysinfo);
        return SYSINFO->GITREF_RP2040;
    }
};

} // namespace brio
