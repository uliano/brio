/*
 * device_tables.hpp
 *
 * THE RESERVE. Everything in this file is a probe of the device
 * header's own per-pad and per-instance symbols - `#ifdef` walls that
 * turn vendor macros into constexpr DATA - and nothing else is allowed
 * to live here.
 *
 * THE RULE THIS FILE ENFORCES (docs/design/overview.md, "Target
 * strata"): a preprocessor conditional whose product is a VALUE - a
 * line number, a clock id, a trigger code, a bonding fact - belongs
 * HERE, quarantined, one entry per header symbol; a driver may keep an
 * `#ifdef` only where it selects CODE that genuinely differs per
 * instance (a register-struct reference, an `if constexpr` tier),
 * which no constexpr table can hold. The drivers stay readable C++23;
 * the one thing C++ cannot do - ask whether a macro exists - happens
 * in one place, with a mandate.
 *
 * WHY PROBES AND NOT PER-VARIANT TABLES. The alternative - three
 * hand-kept (or script-kept) tables selected by one `#if` on the
 * device macro - would be a COPY of the DFP's bonding information,
 * able to drift from it silently. These probes ARE that information,
 * re-read from the header at every compile: a pad the package does not
 * bond has no symbol, so it has no entry, and the driver's
 * static_assert names the absence. Ugly, deliberately confined, and
 * incapable of lying.
 *
 * The maps are irregular and per-package, which is why no formula and
 * no fixed list can stand in for them:
 *  - EIC: PA16 -> EXTINT0, PA24 -> EXTINT12, PA27 -> EXTINT15,
 *    PB30 -> EXTINT14; 25 bonded pads on the E, 37 on the G, 51 on
 *    the J.
 *  - TC waveform outputs: PA22 = TC0/WO0 and PB12 = TC0/WO0 again;
 *    8 pads on the E, 18 on the G, 26 on the J.
 *  - AC analog inputs: AIN6/AIN7 (PB05/PB06) exist on the J alone.
 *
 * Consumers: samc/eic.hpp, samc/tc.hpp, samc/ac.hpp. Each declares the
 * MEANING of its numbers (what an EXTINT line is, what a WO pad does);
 * this file only says which numbers exist on this device.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

namespace brio {

// =============================================================================
// EIC: pad -> external-interrupt line
// =============================================================================
//
// `PIN_P<pad>A_EIC_EXTINT_NUM` exists for exactly the pads a given
// package bonds to the EIC, and its value is that pad's EXTINT number.
// The return is `int` so that "this pad has no external interrupt on
// this device" has a value of its own (-1) that a static_assert can
// name.

#define BRIO_EIC_PAD(letter, number, sym) \
    case (static_cast<int>(letter) - 'A') * 32 + (number): \
        return static_cast<int>(PIN_##sym##A_EIC_EXTINT_NUM);

constexpr int eic_extint_line(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA00A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 0, PA00)
#endif
#ifdef PIN_PA01A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 1, PA01)
#endif
#ifdef PIN_PA02A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 2, PA02)
#endif
#ifdef PIN_PA03A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 3, PA03)
#endif
#ifdef PIN_PA04A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 4, PA04)
#endif
#ifdef PIN_PA05A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 5, PA05)
#endif
#ifdef PIN_PA06A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 6, PA06)
#endif
#ifdef PIN_PA07A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 7, PA07)
#endif
#ifdef PIN_PA09A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 9, PA09)
#endif
#ifdef PIN_PA10A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 10, PA10)
#endif
#ifdef PIN_PA11A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 11, PA11)
#endif
#ifdef PIN_PA12A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 12, PA12)
#endif
#ifdef PIN_PA13A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 13, PA13)
#endif
#ifdef PIN_PA14A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 14, PA14)
#endif
#ifdef PIN_PA15A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 15, PA15)
#endif
#ifdef PIN_PA16A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 16, PA16)
#endif
#ifdef PIN_PA17A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 17, PA17)
#endif
#ifdef PIN_PA18A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 18, PA18)
#endif
#ifdef PIN_PA19A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 19, PA19)
#endif
#ifdef PIN_PA20A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 20, PA20)
#endif
#ifdef PIN_PA21A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 21, PA21)
#endif
#ifdef PIN_PA22A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 22, PA22)
#endif
#ifdef PIN_PA23A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 23, PA23)
#endif
#ifdef PIN_PA24A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 24, PA24)
#endif
#ifdef PIN_PA25A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 25, PA25)
#endif
#ifdef PIN_PA27A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 27, PA27)
#endif
#ifdef PIN_PA28A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 28, PA28)
#endif
#ifdef PIN_PA30A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 30, PA30)
#endif
#ifdef PIN_PA31A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('A', 31, PA31)
#endif
#ifdef PIN_PB00A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 0, PB00)
#endif
#ifdef PIN_PB01A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 1, PB01)
#endif
#ifdef PIN_PB02A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 2, PB02)
#endif
#ifdef PIN_PB03A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 3, PB03)
#endif
#ifdef PIN_PB04A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 4, PB04)
#endif
#ifdef PIN_PB05A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 5, PB05)
#endif
#ifdef PIN_PB06A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 6, PB06)
#endif
#ifdef PIN_PB07A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 7, PB07)
#endif
#ifdef PIN_PB08A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 8, PB08)
#endif
#ifdef PIN_PB09A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 9, PB09)
#endif
#ifdef PIN_PB10A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 10, PB10)
#endif
#ifdef PIN_PB11A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 11, PB11)
#endif
#ifdef PIN_PB12A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 12, PB12)
#endif
#ifdef PIN_PB13A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 13, PB13)
#endif
#ifdef PIN_PB14A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 14, PB14)
#endif
#ifdef PIN_PB15A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 15, PB15)
#endif
#ifdef PIN_PB16A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 16, PB16)
#endif
#ifdef PIN_PB17A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 17, PB17)
#endif
#ifdef PIN_PB22A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 22, PB22)
#endif
#ifdef PIN_PB23A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 23, PB23)
#endif
#ifdef PIN_PB30A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 30, PB30)
#endif
#ifdef PIN_PB31A_EIC_EXTINT_NUM
    BRIO_EIC_PAD('B', 31, PB31)
#endif
    default:
        return -1;
    }
}

#undef BRIO_EIC_PAD

/// Whether a pad is the NMI pad on THIS device, from the same
/// authority (`PIN_P<pad>A_EIC_NMI`). It is PA08 on every variant of
/// this family - but it is the header symbol that says so, not this
/// file.
constexpr bool eic_nmi_pad(char port, uint8_t pin) {
    (void)port;
    (void)pin;
#ifdef PIN_PA08A_EIC_NMI
    if (port == 'A' && pin == 8u) {
        return true;
    }
#endif
    return false;
}

/// Whether a pad carries an external interrupt line on THIS device.
template <char L, uint8_t N>
constexpr bool extint_exists = eic_extint_line(L, N) >= 0;

// =============================================================================
// TC: instance parameters, and pad -> waveform output
// =============================================================================

/// How many TC instances this device has.
constexpr uint8_t tc_count() {
#if defined(TC7_REGS)
    return 8;
#elif defined(TC5_REGS)
    return 6;
#elif defined(TC4_REGS)
    return 5;
#elif defined(TC2_REGS)
    return 3;
#else
    return 0;
#endif
}

/// `TCn_MASTER_SLAVE_MODE`: 1 = can be the master of a 32-bit pair,
/// 2 = is somebody's client, 0 = cannot pair (35.6.2.4).
constexpr uint8_t tc_pair_role(uint8_t n) {
    switch (n) {
#ifdef TC0_MASTER_SLAVE_MODE
    case 0: return TC0_MASTER_SLAVE_MODE;
#endif
#ifdef TC1_MASTER_SLAVE_MODE
    case 1: return TC1_MASTER_SLAVE_MODE;
#endif
#ifdef TC2_MASTER_SLAVE_MODE
    case 2: return TC2_MASTER_SLAVE_MODE;
#endif
#ifdef TC3_MASTER_SLAVE_MODE
    case 3: return TC3_MASTER_SLAVE_MODE;
#endif
#ifdef TC4_MASTER_SLAVE_MODE
    case 4: return TC4_MASTER_SLAVE_MODE;
#endif
    default: return 0;
    }
}

constexpr bool tc_can_pair(uint8_t n) { return tc_pair_role(n) == 1u; }

/// This instance's generic clock channel. TC0/TC1 share one, TC2/TC3
/// share another, TC4 has its own - which is why 35.5.3 warns that two
/// instances "cannot be set to different clock frequencies".
constexpr uint8_t tc_gclk_id(uint8_t n) {
    switch (n) {
#ifdef TC0_GCLK_ID
    case 0: return TC0_GCLK_ID;
#endif
#ifdef TC1_GCLK_ID
    case 1: return TC1_GCLK_ID;
#endif
#ifdef TC2_GCLK_ID
    case 2: return TC2_GCLK_ID;
#endif
#ifdef TC3_GCLK_ID
    case 3: return TC3_GCLK_ID;
#endif
#ifdef TC4_GCLK_ID
    case 4: return TC4_GCLK_ID;
#endif
    default: return 0xFF;
    }
}

/// The DMAC trigger ids of one instance, from the device header's own
/// `TCn_DMAC_ID_*` - dmac.hpp owns the channels and not the trigger
/// table, so the codes live with the peripheral that raises them.
constexpr uint8_t tc_dma_overflow_id(uint8_t n) {
    switch (n) {
#ifdef TC0_DMAC_ID_OVF
    case 0: return TC0_DMAC_ID_OVF;
#endif
#ifdef TC1_DMAC_ID_OVF
    case 1: return TC1_DMAC_ID_OVF;
#endif
#ifdef TC2_DMAC_ID_OVF
    case 2: return TC2_DMAC_ID_OVF;
#endif
#ifdef TC3_DMAC_ID_OVF
    case 3: return TC3_DMAC_ID_OVF;
#endif
#ifdef TC4_DMAC_ID_OVF
    case 4: return TC4_DMAC_ID_OVF;
#endif
    default: return 0;
    }
}

constexpr uint8_t tc_dma_match0_id(uint8_t n) {
    switch (n) {
#ifdef TC0_DMAC_ID_MC0
    case 0: return TC0_DMAC_ID_MC0;
#endif
#ifdef TC1_DMAC_ID_MC0
    case 1: return TC1_DMAC_ID_MC0;
#endif
#ifdef TC2_DMAC_ID_MC0
    case 2: return TC2_DMAC_ID_MC0;
#endif
#ifdef TC3_DMAC_ID_MC0
    case 3: return TC3_DMAC_ID_MC0;
#endif
#ifdef TC4_DMAC_ID_MC0
    case 4: return TC4_DMAC_ID_MC0;
#endif
    default: return 0;
    }
}

// -----------------------------------------------------------------------------
// `PIN_P<pad>E_TC<n>_WO<k>` exists for exactly the pads a package bonds
// to a timer's waveform output, and each pad carries exactly one of
// them. The return packs the instance and the output into one value
// ((tc << 4) | wo), -1 for a pad that carries neither.

#define BRIO_TC_WO_PAD(letter, number, tc, wo) \
    case (static_cast<int>(letter) - 'A') * 32 + (number): \
        return ((tc) << 4) | (wo);

constexpr int tc_wo_code(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA14E_TC4_WO0
    BRIO_TC_WO_PAD('A', 14, 4, 0)
#endif
#ifdef PIN_PA15E_TC4_WO1
    BRIO_TC_WO_PAD('A', 15, 4, 1)
#endif
#ifdef PIN_PA18E_TC4_WO0
    BRIO_TC_WO_PAD('A', 18, 4, 0)
#endif
#ifdef PIN_PA19E_TC4_WO1
    BRIO_TC_WO_PAD('A', 19, 4, 1)
#endif
#ifdef PIN_PA20E_TC3_WO0
    BRIO_TC_WO_PAD('A', 20, 3, 0)
#endif
#ifdef PIN_PA21E_TC3_WO1
    BRIO_TC_WO_PAD('A', 21, 3, 1)
#endif
#ifdef PIN_PA22E_TC0_WO0
    BRIO_TC_WO_PAD('A', 22, 0, 0)
#endif
#ifdef PIN_PA23E_TC0_WO1
    BRIO_TC_WO_PAD('A', 23, 0, 1)
#endif
#ifdef PIN_PA24E_TC1_WO0
    BRIO_TC_WO_PAD('A', 24, 1, 0)
#endif
#ifdef PIN_PA25E_TC1_WO1
    BRIO_TC_WO_PAD('A', 25, 1, 1)
#endif
#ifdef PIN_PB00E_TC3_WO0
    BRIO_TC_WO_PAD('B', 0, 3, 0)
#endif
#ifdef PIN_PB01E_TC3_WO1
    BRIO_TC_WO_PAD('B', 1, 3, 1)
#endif
#ifdef PIN_PB02E_TC2_WO0
    BRIO_TC_WO_PAD('B', 2, 2, 0)
#endif
#ifdef PIN_PB03E_TC2_WO1
    BRIO_TC_WO_PAD('B', 3, 2, 1)
#endif
#ifdef PIN_PB08E_TC0_WO0
    BRIO_TC_WO_PAD('B', 8, 0, 0)
#endif
#ifdef PIN_PB09E_TC0_WO1
    BRIO_TC_WO_PAD('B', 9, 0, 1)
#endif
#ifdef PIN_PB10E_TC1_WO0
    BRIO_TC_WO_PAD('B', 10, 1, 0)
#endif
#ifdef PIN_PB11E_TC1_WO1
    BRIO_TC_WO_PAD('B', 11, 1, 1)
#endif
#ifdef PIN_PB12E_TC0_WO0
    BRIO_TC_WO_PAD('B', 12, 0, 0)
#endif
#ifdef PIN_PB13E_TC0_WO1
    BRIO_TC_WO_PAD('B', 13, 0, 1)
#endif
#ifdef PIN_PB14E_TC1_WO0
    BRIO_TC_WO_PAD('B', 14, 1, 0)
#endif
#ifdef PIN_PB15E_TC1_WO1
    BRIO_TC_WO_PAD('B', 15, 1, 1)
#endif
#ifdef PIN_PB16E_TC2_WO0
    BRIO_TC_WO_PAD('B', 16, 2, 0)
#endif
#ifdef PIN_PB17E_TC2_WO1
    BRIO_TC_WO_PAD('B', 17, 2, 1)
#endif
#ifdef PIN_PB22E_TC3_WO0
    BRIO_TC_WO_PAD('B', 22, 3, 0)
#endif
#ifdef PIN_PB23E_TC3_WO1
    BRIO_TC_WO_PAD('B', 23, 3, 1)
#endif
    default:
        return -1;
    }
}

#undef BRIO_TC_WO_PAD

template <char L, uint8_t N>
constexpr bool tc_wo_exists = tc_wo_code(L, N) >= 0;

// =============================================================================
// AC: which analog inputs this package bonds
// =============================================================================

/// Whether this device bonds a given AC AINx pad at all
/// (`PIN_P<pad>B_AC_AIN<k>`). Which AINx a comparator's PINn code
/// selects is the AC chapter's arithmetic and lives in samc/ac.hpp
/// (the pair owns the pads); this only says which pads exist.
constexpr bool ac_ain_exists(uint8_t ain) {
    switch (ain) {
#ifdef PIN_PA04B_AC_AIN0
    case 0: return true;
#endif
#ifdef PIN_PA05B_AC_AIN1
    case 1: return true;
#endif
#ifdef PIN_PA06B_AC_AIN2
    case 2: return true;
#endif
#ifdef PIN_PA07B_AC_AIN3
    case 3: return true;
#endif
#ifdef PIN_PA02B_AC_AIN4
    case 4: return true;
#endif
#ifdef PIN_PA03B_AC_AIN5
    case 5: return true;
#endif
#ifdef PIN_PB05B_AC_AIN6
    case 6: return true;
#endif
#ifdef PIN_PB06B_AC_AIN7
    case 7: return true;
#endif
    default: return false;
    }
}

} // namespace brio
