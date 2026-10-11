/*
 * pwr.hpp
 *
 * The power controller of the CH32X035 (RM ch. 2), as much of it as a
 * program asks today: the PROGRAMMABLE VOLTAGE DETECTOR (2.2.2), the one
 * witness a board has of its own supply. PWR_CTLR's PLS[1:0] (2.4.1)
 * chooses one of four thresholds and PWR_CSR's PVD0 (2.4.2) says whether
 * VDD stands below it - "1: VDD is below the PVD threshold set by
 * PLS[1:0]". There is NO ENABLE BIT: the register description has none
 * (bits 4:2 of PWR_CTLR are reserved where the STM32F1 keeps PVDE), so the
 * detector is read as it stands, a threshold being chosen.
 *
 * THE FOUR THRESHOLDS (2.4.1), each with its two edges and "approx. 20
 * mV" of hysteresis (figure 2-3): 2.12 V rising / 2.1 V falling, 2.32 /
 * 2.3, 3.02 / 3.0, 4.02 / 4.0. A 3.3 V board reads NOT LOW at the third,
 * which is the level a program that drives a pad into another board's
 * asks at boot: a supply that has sagged under 3.0 V is one that the
 * other board's pads may be feeding through their protection diodes.
 *
 * THE GATE. PWR's registers answer only with the controller's clock open
 * (RCC_APB1PCENR's PWREN); every verb here opens it, idempotent. Its
 * RESET line is clock.hpp's to refuse (docs/ch32x035/clock.md), and
 * nothing here pulses it.
 *
 * WHAT IS NOT HERE: the low-power modes (PDDS, the flash's LP bits) and
 * the PVD's EXTI line 26 - their chapter is not written in this stratum.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/device.hpp"

namespace brio {

inline constexpr uint32_t pwr_pls_shift = 5;
inline constexpr uint32_t pwr_pls_mask = 0x3UL << pwr_pls_shift;
inline constexpr uint32_t pwr_pvd0 = 1UL << 2;   ///< in CSR: VDD below the threshold

/// PLS[1:0], named by the falling edge's volts.
enum class PvdLevel : uint8_t { v2_1 = 0, v2_3 = 1, v3_0 = 2, v4_0 = 3 };

/// The threshold VDD crosses rising, in millivolts (2.4.1).
constexpr uint32_t pvd_rising_mv(PvdLevel level) {
    switch (level) {
        case PvdLevel::v2_1: return 2120;
        case PvdLevel::v2_3: return 2320;
        case PvdLevel::v3_0: return 3020;
        case PvdLevel::v4_0: return 4020;
    }
    return 0;
}

/// The threshold VDD crosses falling, in millivolts (2.4.1).
constexpr uint32_t pvd_falling_mv(PvdLevel level) {
    switch (level) {
        case PvdLevel::v2_1: return 2100;
        case PvdLevel::v2_3: return 2300;
        case PvdLevel::v3_0: return 3000;
        case PvdLevel::v4_0: return 4000;
    }
    return 0;
}

/// The power controller, monostate.
struct Pwr {
    Pwr() = delete;

    /// The controller's clock, opened by every verb (idempotent).
    static void open() { Rcc::enable(Bus::pb1, rcc_pb1_pwr); }

    /// Choose the detector's threshold: PLS written, every other bit of
    /// PWR_CTLR kept. The comparison follows the supply as it stands; a
    /// reader that has just moved the threshold lets the detector settle
    /// before trusting supply_low() (the manual gives no settling time).
    static void pvd(PvdLevel level) {
        open();
        pwr()->CTLR = (pwr()->CTLR & ~pwr_pls_mask) |
                      (static_cast<uint32_t>(level) << pwr_pls_shift);
    }
    static PvdLevel pvd() {
        open();
        return static_cast<PvdLevel>((pwr()->CTLR & pwr_pls_mask) >> pwr_pls_shift);
    }

    /// PVD0: VDD below the chosen threshold.
    static bool supply_low() {
        open();
        return (pwr()->CSR & pwr_pvd0) != 0u;
    }
};

} // namespace brio
