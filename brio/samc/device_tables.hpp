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
 *  - TCC waveform outputs: the same pad carries TWO of them, on two
 *    different peripheral functions - PA08 is TCC0/WO0 under function E
 *    and TCC1/WO2 under function F - so that map is keyed by pad AND
 *    function where the TC's is keyed by pad alone.
 *  - AC analog inputs: AIN6/AIN7 (PB05/PB06) exist on the J alone.
 *  - DAC analog pads: VOUT on PA02 (which is ADC0/AIN0 and AC/AIN4 at
 *    the same time) and VREFA on PA03; one instance, and the C20 half
 *    of the family has no DAC at all.
 *  - ADC analog inputs: TWO maps over OVERLAPPING pads, so that one is
 *    keyed by INSTANCE and pad - PA08 is ADC0/AIN8 and ADC1/AIN10 at
 *    once, PB08 is ADC0/AIN2 and ADC1/AIN4 - and the E bonds no PORT B
 *    pad to either converter at all, which leaves ADC1 there with
 *    exactly AIN10 and AIN11.
 *  - SDADC differential pairs: three PAIRS of pads, and the package
 *    variation is the whole map - the E bonds pair 0 alone (PA06/PA07),
 *    the G adds pair 1 (PB08/PB09), only the J carries pair 2
 *    (PB06/PB07). The external reference pad VREFB (PA04) is on every
 *    variant.
 *
 *  - CCL pads: the whole map is a package fact and one LUT of the four
 *    disappears from the pads entirely - LUT3's IN[9..11] and OUT[3]
 *    are bonded on the J alone, so on the E and the G that LUT exists
 *    (CCL_LUT_NUM is 4 everywhere) with NO pin of its own and must be
 *    reached through events or a link. Two pads also carry the SAME
 *    input: PA04 and PA16 are both CCL0/IN[0], PA08 and PA30 are both
 *    CCL1/IN[3].
 *
 * Consumers: samc/eic.hpp, samc/tc.hpp, samc/tcc.hpp, samc/ac.hpp,
 * samc/adc.hpp, samc/dac.hpp, samc/sdadc.hpp, samc/ccl.hpp. Each declares the MEANING of its numbers
 * (what an EXTINT line is, what a WO pad does); this file only says
 * which numbers exist on this device.
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
// TCC: instance parameters, and (pad, function) -> waveform output
// =============================================================================
//
// The TCC instances are NOT copies of each other the way the TCs are:
// they differ in counter width, in channel count, in output count and in
// which of the five waveform-extension units they implement. Every one
// of those differences is a `TCCn_*` constant of the device header, and
// every one of them is probed here.

/// How many TCC instances this device has.
constexpr uint8_t tcc_count() {
#if defined(TCC2_REGS)
    return 3;
#elif defined(TCC1_REGS)
    return 2;
#elif defined(TCC0_REGS)
    return 1;
#else
    return 0;
#endif
}

/// `TCCn_CC_NUM`: compare/capture channels (4 on TCC0, 2 on TCC1/TCC2).
constexpr uint8_t tcc_cc_count(uint8_t n) {
    switch (n) {
#ifdef TCC0_CC_NUM
    case 0: return TCC0_CC_NUM;
#endif
#ifdef TCC1_CC_NUM
    case 1: return TCC1_CC_NUM;
#endif
#ifdef TCC2_CC_NUM
    case 2: return TCC2_CC_NUM;
#endif
    default: return 0;
    }
}

/// `TCCn_OW_NUM`: waveform outputs (8/4/2). Always at least the channel
/// count, and the output matrix is what fills the difference.
constexpr uint8_t tcc_wo_count(uint8_t n) {
    switch (n) {
#ifdef TCC0_OW_NUM
    case 0: return TCC0_OW_NUM;
#endif
#ifdef TCC1_OW_NUM
    case 1: return TCC1_OW_NUM;
#endif
#ifdef TCC2_OW_NUM
    case 2: return TCC2_OW_NUM;
#endif
    default: return 0;
    }
}

/// `TCCn_SIZE`: the counter's width in BITS - 24 on TCC0 and TCC1, 16 on
/// TCC2. 36.8.15 says the excess bits of a 16-bit instance read zero.
constexpr uint8_t tcc_size(uint8_t n) {
    switch (n) {
#ifdef TCC0_SIZE
    case 0: return TCC0_SIZE;
#endif
#ifdef TCC1_SIZE
    case 1: return TCC1_SIZE;
#endif
#ifdef TCC2_SIZE
    case 2: return TCC2_SIZE;
#endif
    default: return 0;
    }
}

/// This instance's generic clock channel. TCC0 and TCC1 SHARE one
/// (36.5.3 says so in one sentence), TCC2 has its own.
constexpr uint8_t tcc_gclk_id(uint8_t n) {
    switch (n) {
#ifdef TCC0_GCLK_ID
    case 0: return TCC0_GCLK_ID;
#endif
#ifdef TCC1_GCLK_ID
    case 1: return TCC1_GCLK_ID;
#endif
#ifdef TCC2_GCLK_ID
    case 2: return TCC2_GCLK_ID;
#endif
    default: return 0xFF;
    }
}

/// `TCCn_MASTER_SLAVE_MODE`: 1 = can be the HOST of a host/client pair,
/// 2 = is somebody's client (the one that may set CTRLA.MSYNC), 0 =
/// neither (36.6.4).
constexpr uint8_t tcc_pair_role(uint8_t n) {
    switch (n) {
#ifdef TCC0_MASTER_SLAVE_MODE
    case 0: return TCC0_MASTER_SLAVE_MODE;
#endif
#ifdef TCC1_MASTER_SLAVE_MODE
    case 1: return TCC1_MASTER_SLAVE_MODE;
#endif
#ifdef TCC2_MASTER_SLAVE_MODE
    case 2: return TCC2_MASTER_SLAVE_MODE;
#endif
    default: return 0;
    }
}

// ---- the five optional waveform-extension units -----------------------------
//
// 36.6.1: "the following independent units are implemented in some of
// the TCC instances as optional and successive units". Each has its own
// header constant, and a driver asking an instance for one it does not
// have is a compile error rather than a store into a reserved bit.

/// `TCCn_DTI`: dead-time insertion (36.6.3.7).
constexpr bool tcc_has_dead_time(uint8_t n) {
    switch (n) {
#ifdef TCC0_DTI
    case 0: return TCC0_DTI != 0;
#endif
#ifdef TCC1_DTI
    case 1: return TCC1_DTI != 0;
#endif
#ifdef TCC2_DTI
    case 2: return TCC2_DTI != 0;
#endif
    default: return false;
    }
}

/// `TCCn_OTMX`: the output matrix (WEXCTRL.OTMX, table 36-4).
constexpr bool tcc_has_output_matrix(uint8_t n) {
    switch (n) {
#ifdef TCC0_OTMX
    case 0: return TCC0_OTMX != 0;
#endif
#ifdef TCC1_OTMX
    case 1: return TCC1_OTMX != 0;
#endif
#ifdef TCC2_OTMX
    case 2: return TCC2_OTMX != 0;
#endif
    default: return false;
    }
}

/// `TCCn_SWAP`: the DTI output-pair swap (WAVE.SWAPx).
constexpr bool tcc_has_swap(uint8_t n) {
    switch (n) {
#ifdef TCC0_SWAP
    case 0: return TCC0_SWAP != 0;
#endif
#ifdef TCC1_SWAP
    case 1: return TCC1_SWAP != 0;
#endif
#ifdef TCC2_SWAP
    case 2: return TCC2_SWAP != 0;
#endif
    default: return false;
    }
}

/// `TCCn_PG`: pattern generation (PATT/PATTBUF).
constexpr bool tcc_has_pattern(uint8_t n) {
    switch (n) {
#ifdef TCC0_PG
    case 0: return TCC0_PG != 0;
#endif
#ifdef TCC1_PG
    case 1: return TCC1_PG != 0;
#endif
#ifdef TCC2_PG
    case 2: return TCC2_PG != 0;
#endif
    default: return false;
    }
}

/// `TCCn_DITHERING`: the CTRLA.RESOLUTION dithering modes (36.6.3.3).
constexpr bool tcc_has_dithering(uint8_t n) {
    switch (n) {
#ifdef TCC0_DITHERING
    case 0: return TCC0_DITHERING != 0;
#endif
#ifdef TCC1_DITHERING
    case 1: return TCC1_DITHERING != 0;
#endif
#ifdef TCC2_DITHERING
    case 2: return TCC2_DITHERING != 0;
#endif
    default: return false;
    }
}

/// `TCCn_EXT`, the header's own one-word summary of the five units
/// above. It is exported RAW: the five booleans are the authority a
/// driver refuses on, and this is the number they can be cross-checked
/// against (docs/samc/tcc.md carries the decoding the bench confirmed).
constexpr uint8_t tcc_ext_code(uint8_t n) {
    switch (n) {
#ifdef TCC0_EXT
    case 0: return TCC0_EXT;
#endif
#ifdef TCC1_EXT
    case 1: return TCC1_EXT;
#endif
#ifdef TCC2_EXT
    case 2: return TCC2_EXT;
#endif
    default: return 0;
    }
}

// ---- the DMAC trigger ids and the EVSYS codes -------------------------------
//
// dmac.hpp owns the channels and not the trigger table, and evsys.hpp
// owns the fabric and not the vocabulary - so both tables live with the
// peripheral that raises them, probed from the device header's own
// constants rather than counted out by hand. The TCC's generator codes
// are NOT evenly spaced (TCC0 spends seven, TCC1 and TCC2 five each),
// which is exactly why they are read and not computed.

constexpr uint8_t tcc_dma_overflow_id(uint8_t n) {
    switch (n) {
#ifdef TCC0_DMAC_ID_OVF
    case 0: return TCC0_DMAC_ID_OVF;
#endif
#ifdef TCC1_DMAC_ID_OVF
    case 1: return TCC1_DMAC_ID_OVF;
#endif
#ifdef TCC2_DMAC_ID_OVF
    case 2: return TCC2_DMAC_ID_OVF;
#endif
    default: return 0;
    }
}

constexpr uint8_t tcc_dma_match0_id(uint8_t n) {
    switch (n) {
#ifdef TCC0_DMAC_ID_MC0
    case 0: return TCC0_DMAC_ID_MC0;
#endif
#ifdef TCC1_DMAC_ID_MC0
    case 1: return TCC1_DMAC_ID_MC0;
#endif
#ifdef TCC2_DMAC_ID_MC0
    case 2: return TCC2_DMAC_ID_MC0;
#endif
    default: return 0;
    }
}

/// Generator: overflow/underflow. The three counter generators of one
/// instance are consecutive (OVF, TRG, CNT) and the channel generators
/// follow them, so one anchor per instance is enough.
constexpr uint8_t tcc_overflow_generator(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_GEN_TCC0_OVF
    case 0: return EVENT_ID_GEN_TCC0_OVF;
#endif
#ifdef EVENT_ID_GEN_TCC1_OVF
    case 1: return EVENT_ID_GEN_TCC1_OVF;
#endif
#ifdef EVENT_ID_GEN_TCC2_OVF
    case 2: return EVENT_ID_GEN_TCC2_OVF;
#endif
    default: return 0;
    }
}

/// Generator: match or capture on channel 0.
constexpr uint8_t tcc_match0_generator(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_GEN_TCC0_MC_0
    case 0: return EVENT_ID_GEN_TCC0_MC_0;
#endif
#ifdef EVENT_ID_GEN_TCC1_MC_0
    case 1: return EVENT_ID_GEN_TCC1_MC_0;
#endif
#ifdef EVENT_ID_GEN_TCC2_MC_0
    case 2: return EVENT_ID_GEN_TCC2_MC_0;
#endif
    default: return 0;
    }
}

/// User: the first of this instance's two counter event inputs (EV0).
constexpr uint8_t tcc_event0_user(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_USER_TCC0_EV_0
    case 0: return EVENT_ID_USER_TCC0_EV_0;
#endif
#ifdef EVENT_ID_USER_TCC1_EV_0
    case 1: return EVENT_ID_USER_TCC1_EV_0;
#endif
#ifdef EVENT_ID_USER_TCC2_EV_0
    case 2: return EVENT_ID_USER_TCC2_EV_0;
#endif
    default: return 0;
    }
}

/// User: this instance's channel-0 event input (MC0) - which is also
/// recoverable Fault A's source (36.6.3.5).
constexpr uint8_t tcc_match0_user(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_USER_TCC0_MC_0
    case 0: return EVENT_ID_USER_TCC0_MC_0;
#endif
#ifdef EVENT_ID_USER_TCC1_MC_0
    case 1: return EVENT_ID_USER_TCC1_MC_0;
#endif
#ifdef EVENT_ID_USER_TCC2_MC_0
    case 2: return EVENT_ID_USER_TCC2_MC_0;
#endif
    default: return 0;
    }
}

// -----------------------------------------------------------------------------
// `PIN_P<pad><fn>_TCC<n>_WO<k>` exists for exactly the (pad, peripheral
// function) pairs a package bonds to a TCC waveform output. UNLIKE the
// TC's map this one needs the FUNCTION as a key: PA08 carries TCC0/WO0
// under function E and TCC1/WO2 under function F, and a driver that
// took only the pad would have to guess which. The return packs the
// instance and the output into one value ((tcc << 4) | wo), -1 for a
// pair that carries none.

#define BRIO_TCC_WO_PAD(letter, number, tcc, wo) \
    case (static_cast<int>(letter) - 'A') * 32 + (number): \
        return ((tcc) << 4) | (wo);

constexpr int tcc_wo_code_e(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA00E_TCC2_WO0
    BRIO_TCC_WO_PAD('A', 0, 2, 0)
#endif
#ifdef PIN_PA01E_TCC2_WO1
    BRIO_TCC_WO_PAD('A', 1, 2, 1)
#endif
#ifdef PIN_PA04E_TCC0_WO0
    BRIO_TCC_WO_PAD('A', 4, 0, 0)
#endif
#ifdef PIN_PA05E_TCC0_WO1
    BRIO_TCC_WO_PAD('A', 5, 0, 1)
#endif
#ifdef PIN_PA06E_TCC1_WO0
    BRIO_TCC_WO_PAD('A', 6, 1, 0)
#endif
#ifdef PIN_PA07E_TCC1_WO1
    BRIO_TCC_WO_PAD('A', 7, 1, 1)
#endif
#ifdef PIN_PA08E_TCC0_WO0
    BRIO_TCC_WO_PAD('A', 8, 0, 0)
#endif
#ifdef PIN_PA09E_TCC0_WO1
    BRIO_TCC_WO_PAD('A', 9, 0, 1)
#endif
#ifdef PIN_PA10E_TCC1_WO0
    BRIO_TCC_WO_PAD('A', 10, 1, 0)
#endif
#ifdef PIN_PA11E_TCC1_WO1
    BRIO_TCC_WO_PAD('A', 11, 1, 1)
#endif
#ifdef PIN_PA12E_TCC2_WO0
    BRIO_TCC_WO_PAD('A', 12, 2, 0)
#endif
#ifdef PIN_PA13E_TCC2_WO1
    BRIO_TCC_WO_PAD('A', 13, 2, 1)
#endif
#ifdef PIN_PA16E_TCC2_WO0
    BRIO_TCC_WO_PAD('A', 16, 2, 0)
#endif
#ifdef PIN_PA17E_TCC2_WO1
    BRIO_TCC_WO_PAD('A', 17, 2, 1)
#endif
#ifdef PIN_PA30E_TCC1_WO0
    BRIO_TCC_WO_PAD('A', 30, 1, 0)
#endif
#ifdef PIN_PA31E_TCC1_WO1
    BRIO_TCC_WO_PAD('A', 31, 1, 1)
#endif
#ifdef PIN_PB30E_TCC0_WO0
    BRIO_TCC_WO_PAD('B', 30, 0, 0)
#endif
#ifdef PIN_PB31E_TCC0_WO1
    BRIO_TCC_WO_PAD('B', 31, 0, 1)
#endif
    default:
        return -1;
    }
}

constexpr int tcc_wo_code_f(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA08F_TCC1_WO2
    BRIO_TCC_WO_PAD('A', 8, 1, 2)
#endif
#ifdef PIN_PA09F_TCC1_WO3
    BRIO_TCC_WO_PAD('A', 9, 1, 3)
#endif
#ifdef PIN_PA10F_TCC0_WO2
    BRIO_TCC_WO_PAD('A', 10, 0, 2)
#endif
#ifdef PIN_PA11F_TCC0_WO3
    BRIO_TCC_WO_PAD('A', 11, 0, 3)
#endif
#ifdef PIN_PA12F_TCC0_WO6
    BRIO_TCC_WO_PAD('A', 12, 0, 6)
#endif
#ifdef PIN_PA13F_TCC0_WO7
    BRIO_TCC_WO_PAD('A', 13, 0, 7)
#endif
#ifdef PIN_PA14F_TCC0_WO4
    BRIO_TCC_WO_PAD('A', 14, 0, 4)
#endif
#ifdef PIN_PA15F_TCC0_WO5
    BRIO_TCC_WO_PAD('A', 15, 0, 5)
#endif
#ifdef PIN_PA16F_TCC0_WO6
    BRIO_TCC_WO_PAD('A', 16, 0, 6)
#endif
#ifdef PIN_PA17F_TCC0_WO7
    BRIO_TCC_WO_PAD('A', 17, 0, 7)
#endif
#ifdef PIN_PA18F_TCC0_WO2
    BRIO_TCC_WO_PAD('A', 18, 0, 2)
#endif
#ifdef PIN_PA19F_TCC0_WO3
    BRIO_TCC_WO_PAD('A', 19, 0, 3)
#endif
#ifdef PIN_PA20F_TCC0_WO6
    BRIO_TCC_WO_PAD('A', 20, 0, 6)
#endif
#ifdef PIN_PA21F_TCC0_WO7
    BRIO_TCC_WO_PAD('A', 21, 0, 7)
#endif
#ifdef PIN_PA22F_TCC0_WO4
    BRIO_TCC_WO_PAD('A', 22, 0, 4)
#endif
#ifdef PIN_PA23F_TCC0_WO5
    BRIO_TCC_WO_PAD('A', 23, 0, 5)
#endif
#ifdef PIN_PA24F_TCC1_WO2
    BRIO_TCC_WO_PAD('A', 24, 1, 2)
#endif
#ifdef PIN_PA25F_TCC1_WO3
    BRIO_TCC_WO_PAD('A', 25, 1, 3)
#endif
#ifdef PIN_PB10F_TCC0_WO4
    BRIO_TCC_WO_PAD('B', 10, 0, 4)
#endif
#ifdef PIN_PB11F_TCC0_WO5
    BRIO_TCC_WO_PAD('B', 11, 0, 5)
#endif
#ifdef PIN_PB12F_TCC0_WO6
    BRIO_TCC_WO_PAD('B', 12, 0, 6)
#endif
#ifdef PIN_PB13F_TCC0_WO7
    BRIO_TCC_WO_PAD('B', 13, 0, 7)
#endif
#ifdef PIN_PB16F_TCC0_WO4
    BRIO_TCC_WO_PAD('B', 16, 0, 4)
#endif
#ifdef PIN_PB17F_TCC0_WO5
    BRIO_TCC_WO_PAD('B', 17, 0, 5)
#endif
#ifdef PIN_PB30F_TCC1_WO2
    BRIO_TCC_WO_PAD('B', 30, 1, 2)
#endif
#ifdef PIN_PB31F_TCC1_WO3
    BRIO_TCC_WO_PAD('B', 31, 1, 3)
#endif
    default:
        return -1;
    }
}

#undef BRIO_TCC_WO_PAD

/// The two maps behind one lookup. `function` is the PMUX letter in
/// lower case ('e' or 'f'); anything else has no TCC output by
/// definition, because no other function carries one on this family.
constexpr int tcc_wo_code(char port, uint8_t pin, char function) {
    if (function == 'e') {
        return tcc_wo_code_e(port, pin);
    }
    if (function == 'f') {
        return tcc_wo_code_f(port, pin);
    }
    return -1;
}

template <char L, uint8_t N, char F>
constexpr bool tcc_wo_exists = tcc_wo_code(L, N, F) >= 0;

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

// =============================================================================
// ADC: instance parameters, and (instance, pad) -> analog input number
// =============================================================================
//
// The two converters are NOT copies: ADC0 is the host of the host/client
// pair and ADC1 the client, and their pad maps are DIFFERENT MAPS OVER
// OVERLAPPING PADS - PA08 is ADC0/AIN8 and ADC1/AIN10 at the same time,
// PB08 is ADC0/AIN2 and ADC1/AIN4. So this map is keyed by INSTANCE and
// pad, the way the TCC's is keyed by function and pad, and for the same
// reason: a lookup on the pad alone would have to guess.

/// How many ADC instances this device has.
constexpr uint8_t adc_count() {
#if defined(ADC1_REGS)
    return 2;
#elif defined(ADC0_REGS)
    return 1;
#else
    return 0;
#endif
}

/// `ADCn_MASTER_SLAVE_MODE`: 1 = the HOST of the pair (38.6.3.1, the one
/// that owns DUALSEL), 2 = the CLIENT (the one that may set
/// CTRLA.SLAVEEN), 0 = neither.
constexpr uint8_t adc_pair_role(uint8_t n) {
    switch (n) {
#ifdef ADC0_MASTER_SLAVE_MODE
    case 0: return ADC0_MASTER_SLAVE_MODE;
#endif
#ifdef ADC1_MASTER_SLAVE_MODE
    case 1: return ADC1_MASTER_SLAVE_MODE;
#endif
    default: return 0;
    }
}

/// `ADCn_EXTCHANNEL_MSB` + 1: how many EXTERNAL channels (AIN codes) the
/// mux can name. It is the CODE SPACE and not the bonding - which pads a
/// package actually carries is `adc_ain_code()` below.
constexpr uint8_t adc_external_channels(uint8_t n) {
    switch (n) {
#ifdef ADC0_EXTCHANNEL_MSB
    case 0: return static_cast<uint8_t>(ADC0_EXTCHANNEL_MSB + 1);
#endif
#ifdef ADC1_EXTCHANNEL_MSB
    case 1: return static_cast<uint8_t>(ADC1_EXTCHANNEL_MSB + 1);
#endif
    default: return 0;
    }
}

/// `ADCn_LOAD_CALIB`: the header's own statement that this instance's
/// CALIB register has to be loaded from the NVM software calibration row
/// (38.5.10). It reads 1 on both instances of this family, and the
/// driver's init() refuses to run uncalibrated where it does.
constexpr bool adc_loads_calibration(uint8_t n) {
    switch (n) {
#ifdef ADC0_LOAD_CALIB
    case 0: return ADC0_LOAD_CALIB != 0;
#endif
#ifdef ADC1_LOAD_CALIB
    case 1: return ADC1_LOAD_CALIB != 0;
#endif
    default: return false;
    }
}

/// This instance's generic clock channel (GCLK_ADCn).
constexpr uint8_t adc_gclk_id(uint8_t n) {
    switch (n) {
#ifdef ADC0_GCLK_ID
    case 0: return ADC0_GCLK_ID;
#endif
#ifdef ADC1_GCLK_ID
    case 1: return ADC1_GCLK_ID;
#endif
    default: return 0xFF;
    }
}

/// The DMAC trigger id of this instance's one DMA request (RESRDY).
constexpr uint8_t adc_dma_resrdy_id(uint8_t n) {
    switch (n) {
#ifdef ADC0_DMAC_ID_RESRDY
    case 0: return ADC0_DMAC_ID_RESRDY;
#endif
#ifdef ADC1_DMAC_ID_RESRDY
    case 1: return ADC1_DMAC_ID_RESRDY;
#endif
    default: return 0;
    }
}

// ---- the EVSYS codes, from the device header's own constants ----------------

constexpr uint8_t adc_resrdy_generator(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_GEN_ADC0_RESRDY
    case 0: return EVENT_ID_GEN_ADC0_RESRDY;
#endif
#ifdef EVENT_ID_GEN_ADC1_RESRDY
    case 1: return EVENT_ID_GEN_ADC1_RESRDY;
#endif
    default: return 0;
    }
}

constexpr uint8_t adc_winmon_generator(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_GEN_ADC0_WINMON
    case 0: return EVENT_ID_GEN_ADC0_WINMON;
#endif
#ifdef EVENT_ID_GEN_ADC1_WINMON
    case 1: return EVENT_ID_GEN_ADC1_WINMON;
#endif
    default: return 0;
    }
}

constexpr uint8_t adc_start_user(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_USER_ADC0_START
    case 0: return EVENT_ID_USER_ADC0_START;
#endif
#ifdef EVENT_ID_USER_ADC1_START
    case 1: return EVENT_ID_USER_ADC1_START;
#endif
    default: return 0;
    }
}

/// The FLUSH event user. The chapter calls this input FLUSH (38.6.6 and
/// EVCTRL.FLUSHEI) and table 29-3's own row calls it "ADCn SYNC / Flush
/// ADC" - the device header follows the table, so the SYMBOL is
/// `EVENT_ID_USER_ADCn_SYNC` and the MEANING is flush.
constexpr uint8_t adc_flush_user(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_USER_ADC0_SYNC
    case 0: return EVENT_ID_USER_ADC0_SYNC;
#endif
#ifdef EVENT_ID_USER_ADC1_SYNC
    case 1: return EVENT_ID_USER_ADC1_SYNC;
#endif
    default: return 0;
    }
}

// -----------------------------------------------------------------------------
// `PIN_P<pad>B_ADC<n>_AIN<k>` exists for exactly the pads a package bonds
// to a converter's analog input, and its presence is the only authority
// on which AIN codes a variant can actually reach. The E bonds NO PORT B
// pad to either converter, so ADC1 there has AIN10 and AIN11 and nothing
// else; the G bonds six of ADC1's twelve; only the J bonds them all.

#define BRIO_ADC_AIN_PAD(letter, number, ain) \
    case (static_cast<int>(letter) - 'A') * 32 + (number): \
        return (ain);

constexpr int adc0_ain_code(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA02B_ADC0_AIN0
    BRIO_ADC_AIN_PAD('A', 2, 0)
#endif
#ifdef PIN_PA03B_ADC0_AIN1
    BRIO_ADC_AIN_PAD('A', 3, 1)
#endif
#ifdef PIN_PB08B_ADC0_AIN2
    BRIO_ADC_AIN_PAD('B', 8, 2)
#endif
#ifdef PIN_PB09B_ADC0_AIN3
    BRIO_ADC_AIN_PAD('B', 9, 3)
#endif
#ifdef PIN_PA04B_ADC0_AIN4
    BRIO_ADC_AIN_PAD('A', 4, 4)
#endif
#ifdef PIN_PA05B_ADC0_AIN5
    BRIO_ADC_AIN_PAD('A', 5, 5)
#endif
#ifdef PIN_PA06B_ADC0_AIN6
    BRIO_ADC_AIN_PAD('A', 6, 6)
#endif
#ifdef PIN_PA07B_ADC0_AIN7
    BRIO_ADC_AIN_PAD('A', 7, 7)
#endif
#ifdef PIN_PA08B_ADC0_AIN8
    BRIO_ADC_AIN_PAD('A', 8, 8)
#endif
#ifdef PIN_PA09B_ADC0_AIN9
    BRIO_ADC_AIN_PAD('A', 9, 9)
#endif
#ifdef PIN_PA10B_ADC0_AIN10
    BRIO_ADC_AIN_PAD('A', 10, 10)
#endif
#ifdef PIN_PA11B_ADC0_AIN11
    BRIO_ADC_AIN_PAD('A', 11, 11)
#endif
    default:
        return -1;
    }
}

constexpr int adc1_ain_code(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PB00B_ADC1_AIN0
    BRIO_ADC_AIN_PAD('B', 0, 0)
#endif
#ifdef PIN_PB01B_ADC1_AIN1
    BRIO_ADC_AIN_PAD('B', 1, 1)
#endif
#ifdef PIN_PB02B_ADC1_AIN2
    BRIO_ADC_AIN_PAD('B', 2, 2)
#endif
#ifdef PIN_PB03B_ADC1_AIN3
    BRIO_ADC_AIN_PAD('B', 3, 3)
#endif
#ifdef PIN_PB08B_ADC1_AIN4
    BRIO_ADC_AIN_PAD('B', 8, 4)
#endif
#ifdef PIN_PB09B_ADC1_AIN5
    BRIO_ADC_AIN_PAD('B', 9, 5)
#endif
#ifdef PIN_PB04B_ADC1_AIN6
    BRIO_ADC_AIN_PAD('B', 4, 6)
#endif
#ifdef PIN_PB05B_ADC1_AIN7
    BRIO_ADC_AIN_PAD('B', 5, 7)
#endif
#ifdef PIN_PB06B_ADC1_AIN8
    BRIO_ADC_AIN_PAD('B', 6, 8)
#endif
#ifdef PIN_PB07B_ADC1_AIN9
    BRIO_ADC_AIN_PAD('B', 7, 9)
#endif
#ifdef PIN_PA08B_ADC1_AIN10
    BRIO_ADC_AIN_PAD('A', 8, 10)
#endif
#ifdef PIN_PA09B_ADC1_AIN11
    BRIO_ADC_AIN_PAD('A', 9, 11)
#endif
    default:
        return -1;
    }
}

#undef BRIO_ADC_AIN_PAD

/// The two maps behind one lookup: which AIN code this pad is FOR THIS
/// CONVERTER, or -1 if this package does not bond it to that one.
constexpr int adc_ain_code(uint8_t instance, char port, uint8_t pin) {
    if (instance == 0u) {
        return adc0_ain_code(port, pin);
    }
    if (instance == 1u) {
        return adc1_ain_code(port, pin);
    }
    return -1;
}

/// Whether an AIN code of an instance reaches any pad on THIS device -
/// derived from the map above rather than probed a second time, so the
/// two answers cannot disagree.
constexpr bool adc_ain_exists(uint8_t instance, uint8_t ain) {
    for (uint8_t p = 0; p < 2u; ++p) {
        const char port = static_cast<char>('A' + p);
        for (uint8_t i = 0; i < 32u; ++i) {
            if (adc_ain_code(instance, port, i) == static_cast<int>(ain)) {
                return true;
            }
        }
    }
    return false;
}

template <uint8_t I, char L, uint8_t N>
constexpr bool adc_ain_pad_exists = adc_ain_code(I, L, N) >= 0;

// =============================================================================
// DAC: the single instance's parameters and its two analog pads
// =============================================================================
//
// One instance on every C21 variant, so nothing here is keyed by number;
// the probes exist because the C20 half of the family has no DAC at all
// (ch. 41 is titled "SAM C21 only") and because a driver must never copy
// a clock id or a trigger code out of a datasheet table by hand.

/// How many DAC instances this device has: one on every C21, none on a
/// C20 (whose headers this repository does not vendor - the probe is
/// what makes that a compile-time fact rather than an assumption).
constexpr uint8_t dac_count() {
#if defined(DAC_REGS)
    return 1;
#else
    return 0;
#endif
}

/// GCLK_DAC's peripheral channel.
constexpr uint8_t dac_gclk_id() {
#ifdef DAC_GCLK_ID
    return DAC_GCLK_ID;
#else
    return 0xFF;
#endif
}

/// The DMAC trigger id of the one DMA request this peripheral has
/// (EMPTY, 41.6.3).
constexpr uint8_t dac_dma_empty_id() {
#ifdef DAC_DMAC_ID_EMPTY
    return DAC_DMAC_ID_EMPTY;
#else
    return 0;
#endif
}

/// EVSYS generator: the data buffer became empty.
constexpr uint8_t dac_empty_generator() {
#ifdef EVENT_ID_GEN_DAC_EMPTY
    return EVENT_ID_GEN_DAC_EMPTY;
#else
    return 0;
#endif
}

/// EVSYS user: start a conversion from DATABUF. Table 29-3 marks it
/// ASYNCHRONOUS PATH ONLY.
constexpr uint8_t dac_start_user() {
#ifdef EVENT_ID_USER_DAC_START
    return EVENT_ID_USER_DAC_START;
#else
    return 0;
#endif
}

// -----------------------------------------------------------------------------
// The two analog pads. `PIN_P<pad>B_DAC_VOUT` and `..._DAC_VREFP` exist
// for exactly the pads a package bonds, and the matching `MUX_` symbol is
// the peripheral FUNCTION the pad must be given (B on this family). The
// return is that function index, or -1 for a pad that is not the one.
//
// Both are on PORT A pads every variant bonds - PA02 and PA03 - so no
// package variation is expected here; the probes are what makes that a
// re-read fact instead of a claim.

constexpr int dac_vout_code(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA02B_DAC_VOUT
    case 2: return static_cast<int>(MUX_PA02B_DAC_VOUT);
#endif
    default: return -1;
    }
}

constexpr int dac_vrefa_code(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA03B_DAC_VREFP
    case 3: return static_cast<int>(MUX_PA03B_DAC_VREFP);
#endif
    default: return -1;
    }
}

template <char L, uint8_t N>
constexpr bool dac_vout_pad_exists = dac_vout_code(L, N) >= 0;
template <char L, uint8_t N>
constexpr bool dac_vrefa_pad_exists = dac_vrefa_code(L, N) >= 0;

// =============================================================================
// SDADC: the single instance's parameters and its three PAD PAIRS
// =============================================================================
//
// One instance on every C21 variant (ch. 39 is titled "SAM C21 only"), so
// nothing here is keyed by number - but the pads are keyed by PAIR and by
// POLARITY, because an SDADC input is not a pad, it is two pads. The
// device header spells them INN<k> / INP<k> where the datasheet's own
// signal table says AINN<k> / AINP<k>; the symbols are the header's.
//
// THE PACKAGE VARIATION IS THE WHOLE MAP: pair 0 (PA06/PA07) is on every
// variant, pair 1 (PB08/PB09) is absent on the E, pair 2 (PB06/PB07)
// exists on the J alone. So a suite that converts on pair 2 compiles for
// exactly one package, and `sdadc_pair_exists()` is what says so.

/// How many SDADC instances this device has: one on every C21, none on a
/// C20 (whose headers this repository does not vendor - the probe is
/// what makes that a compile-time fact rather than an assumption).
constexpr uint8_t sdadc_count() {
#if defined(SDADC_REGS)
    return 1;
#else
    return 0;
#endif
}

/// `SDADC_EXT_CHANNELS`: how many differential pairs the MUXSEL field can
/// NAME. It is the code space and not the bonding - which pairs a package
/// actually carries is `sdadc_pair_exists()` below.
constexpr uint8_t sdadc_channels() {
#ifdef SDADC_EXT_CHANNELS
    return SDADC_EXT_CHANNELS;
#else
    return 0;
#endif
}

/// GCLK_SDADC's peripheral channel.
constexpr uint8_t sdadc_gclk_id() {
#ifdef SDADC_GCLK_ID
    return SDADC_GCLK_ID;
#else
    return 0xFF;
#endif
}

/// The DMAC trigger id of the one DMA request this peripheral has
/// (RESRDY, 39.6.4).
constexpr uint8_t sdadc_dma_resrdy_id() {
#ifdef SDADC_DMAC_ID_RESRDY
    return SDADC_DMAC_ID_RESRDY;
#else
    return 0;
#endif
}

/// EVSYS generator: a conversion result is available.
constexpr uint8_t sdadc_resrdy_generator() {
#ifdef EVENT_ID_GEN_SDADC_RESRDY
    return EVENT_ID_GEN_SDADC_RESRDY;
#else
    return 0;
#endif
}

/// EVSYS generator: the window monitor's condition matched.
constexpr uint8_t sdadc_winmon_generator() {
#ifdef EVENT_ID_GEN_SDADC_WINMON
    return EVENT_ID_GEN_SDADC_WINMON;
#else
    return 0;
#endif
}

/// EVSYS user: start a conversion. 39.6.6 says the SDADC uses only
/// ASYNCHRONOUS events.
constexpr uint8_t sdadc_start_user() {
#ifdef EVENT_ID_USER_SDADC_START
    return EVENT_ID_USER_SDADC_START;
#else
    return 0;
#endif
}

/// EVSYS user: flush the pipeline and restart. Same path restriction.
constexpr uint8_t sdadc_flush_user() {
#ifdef EVENT_ID_USER_SDADC_FLUSH
    return EVENT_ID_USER_SDADC_FLUSH;
#else
    return 0;
#endif
}

// -----------------------------------------------------------------------------
// The pads. A pad is encoded the way every switch key in this file is -
// (port - 'A') * 32 + pin - so ONE int carries the whole coordinate and
// the caller unpacks it with the two helpers below.

/// Unpack a packed pad coordinate into its port letter.
constexpr char sdadc_pad_port(int pad) {
    return pad < 0 ? '\0' : static_cast<char>('A' + pad / 32);
}
/// Unpack a packed pad coordinate into its pin number.
constexpr uint8_t sdadc_pad_pin(int pad) {
    return pad < 0 ? 0xFFu : static_cast<uint8_t>(pad % 32);
}

/// Which pad is the NEGATIVE input of a pair on this package, packed, or
/// -1 if this package does not bond it.
constexpr int sdadc_negative_pad(uint8_t pair) {
    switch (pair) {
#ifdef PIN_PA06B_SDADC_INN0
    case 0: return 0 * 32 + 6;
#endif
#ifdef PIN_PB08B_SDADC_INN1
    case 1: return 1 * 32 + 8;
#endif
#ifdef PIN_PB06B_SDADC_INN2
    case 2: return 1 * 32 + 6;
#endif
    default: return -1;
    }
}

/// The same for the POSITIVE input of a pair.
constexpr int sdadc_positive_pad(uint8_t pair) {
    switch (pair) {
#ifdef PIN_PA07B_SDADC_INP0
    case 0: return 0 * 32 + 7;
#endif
#ifdef PIN_PB09B_SDADC_INP1
    case 1: return 1 * 32 + 9;
#endif
#ifdef PIN_PB07B_SDADC_INP2
    case 2: return 1 * 32 + 7;
#endif
    default: return -1;
    }
}

/// A pair is usable only if BOTH its pads are bonded - derived from the
/// two maps rather than probed a third time, so the answers cannot
/// disagree.
constexpr bool sdadc_pair_exists(uint8_t pair) {
    return sdadc_negative_pad(pair) >= 0 && sdadc_positive_pad(pair) >= 0;
}

/// The peripheral FUNCTION a pad needs to be an SDADC negative input (B
/// on this family, taken from the header's own MUX_ symbol), or -1 if it
/// is not one on this package.
constexpr int sdadc_inn_code(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA06B_SDADC_INN0
    case 0 * 32 + 6: return static_cast<int>(MUX_PA06B_SDADC_INN0);
#endif
#ifdef PIN_PB08B_SDADC_INN1
    case 1 * 32 + 8: return static_cast<int>(MUX_PB08B_SDADC_INN1);
#endif
#ifdef PIN_PB06B_SDADC_INN2
    case 1 * 32 + 6: return static_cast<int>(MUX_PB06B_SDADC_INN2);
#endif
    default: return -1;
    }
}

/// The same for a positive input.
constexpr int sdadc_inp_code(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA07B_SDADC_INP0
    case 0 * 32 + 7: return static_cast<int>(MUX_PA07B_SDADC_INP0);
#endif
#ifdef PIN_PB09B_SDADC_INP1
    case 1 * 32 + 9: return static_cast<int>(MUX_PB09B_SDADC_INP1);
#endif
#ifdef PIN_PB07B_SDADC_INP2
    case 1 * 32 + 7: return static_cast<int>(MUX_PB07B_SDADC_INP2);
#endif
    default: return -1;
    }
}

/// The external reference pad. The datasheet's signal table and REFCTRL
/// call it VREFB; the device header calls the pad symbol VREFP. Same pad
/// (PA04), and it is bonded on every variant.
constexpr int sdadc_vrefb_code(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA04B_SDADC_VREFP
    case 4: return static_cast<int>(MUX_PA04B_SDADC_VREFP);
#endif
    default: return -1;
    }
}

template <uint8_t Pair>
constexpr bool sdadc_pair_bonded = sdadc_pair_exists(Pair);
template <char L, uint8_t N>
constexpr bool sdadc_inn_pad_exists = sdadc_inn_code(L, N) >= 0;
template <char L, uint8_t N>
constexpr bool sdadc_inp_pad_exists = sdadc_inp_code(L, N) >= 0;
template <char L, uint8_t N>
constexpr bool sdadc_vrefb_pad_exists = sdadc_vrefb_code(L, N) >= 0;

// =============================================================================
// TSENS: the single instance's parameters
// =============================================================================
//
// One instance on every C21 variant (ch. 43 is a C21-only chapter, and
// table 1-1's note adds that TSENS is absent from the AEC-Q100 qualified
// part numbers - a marking difference the device headers do not express,
// so the probe below answers only "does this header declare the block").
// There are NO PADS: 43.5.1 is "Not applicable", the sensor is entirely
// on the die.

/// How many TSENS instances this device has.
constexpr uint8_t tsens_count() {
#if defined(TSENS_REGS)
    return 1;
#else
    return 0;
#endif
}

/// GCLK_TSENS's peripheral channel. UNLIKE EVERY OTHER PERIPHERAL CHANNEL
/// IN THIS FILE, the generator behind it is part of the measurement's
/// arithmetic and not merely its pace - see samc/tsens.hpp.
constexpr uint8_t tsens_gclk_id() {
#ifdef TSENS_GCLK_ID
    return TSENS_GCLK_ID;
#else
    return 0xFF;
#endif
}

/// The DMAC trigger id of the one DMA request this peripheral has
/// (RESRDY, 43.6.3).
constexpr uint8_t tsens_dma_resrdy_id() {
#ifdef TSENS_DMAC_ID_RESRDY
    return TSENS_DMAC_ID_RESRDY;
#else
    return 0;
#endif
}

/// EVSYS generator: the window monitor's condition matched.
constexpr uint8_t tsens_winmon_generator() {
#ifdef EVENT_ID_GEN_TSENS_WINMON
    return EVENT_ID_GEN_TSENS_WINMON;
#else
    return 0;
#endif
}

/// EVSYS user: start a measurement. It is USER 0 - the first row of
/// table 29-3 - and that row grants all three propagation paths, where
/// the DAC's and the SDADC's START users are asynchronous-only.
constexpr uint8_t tsens_start_user() {
#ifdef EVENT_ID_USER_TSENS_START
    return EVENT_ID_USER_TSENS_START;
#else
    return 0;
#endif
}

/// The PAC peripheral identifier (PAC.WRCTRL.PERID). Erratum 1.19.1 is
/// about this number: with write protection set for it, TSENS.CTRLB
/// writes stop working although 43.5.8 lists CTRLB as unprotectable.
/// There is no PAC driver in this stratum yet; the id is published so
/// the pass that writes one inherits the fact.
constexpr uint16_t tsens_pac_id() {
#ifdef ID_TSENS
    return ID_TSENS;
#else
    return 0xFFFF;
#endif
}

// =============================================================================
// CCL: instance parameters, the EVSYS vocabulary, and the pad maps
// =============================================================================

/// How many look-up tables this device implements (`CCL_LUT_NUM`).
constexpr uint8_t ccl_lut_count() {
#ifdef CCL_LUT_NUM
    return CCL_LUT_NUM;
#else
    return 0;
#endif
}

/// How many sequential sub-modules (`CCL_SEQ_NUM`): one per LUT PAIR.
constexpr uint8_t ccl_seq_count() {
#ifdef CCL_SEQ_NUM
    return CCL_SEQ_NUM;
#else
    return 0;
#endif
}

/// How many LUT input LINES the peripheral has (`CCL_IO_NUM`) - three
/// per LUT, numbered IN[0..11] across the whole block, which is the
/// numbering the pad map below speaks.
constexpr uint8_t ccl_io_count() {
#ifdef CCL_IO_NUM
    return CCL_IO_NUM;
#else
    return 0;
#endif
}

/// GCLK_CCL's peripheral channel - ONE channel for the whole block, so
/// every filter, edge detector and sequencer in it runs at one rate.
constexpr uint8_t ccl_gclk_id() {
#ifdef CCL_GCLK_ID
    return CCL_GCLK_ID;
#else
    return 0xFF;
#endif
}

/// The PAC peripheral identifier (bridge = id / 32, STATUS bit = id %
/// 32). Erratum 1.7.4 is about this number: writing CTRL.SWRST is said
/// to raise a PAC protection error on every revision of this family.
constexpr uint16_t ccl_pac_id() {
#ifdef ID_CCL
    return ID_CCL;
#else
    return 0xFFFF;
#endif
}

/// EVSYS generator: LUT n's output value.
constexpr uint8_t ccl_lutout_generator(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_GEN_CCL_LUTOUT_0
    case 0: return EVENT_ID_GEN_CCL_LUTOUT_0;
#endif
#ifdef EVENT_ID_GEN_CCL_LUTOUT_1
    case 1: return EVENT_ID_GEN_CCL_LUTOUT_1;
#endif
#ifdef EVENT_ID_GEN_CCL_LUTOUT_2
    case 2: return EVENT_ID_GEN_CCL_LUTOUT_2;
#endif
#ifdef EVENT_ID_GEN_CCL_LUTOUT_3
    case 3: return EVENT_ID_GEN_CCL_LUTOUT_3;
#endif
    default: return 0;
    }
}

/// EVSYS user: LUT n's one event input line. Table 29-3 marks all four
/// ASYNCHRONOUS PATH ONLY.
constexpr uint8_t ccl_lutin_user(uint8_t n) {
    switch (n) {
#ifdef EVENT_ID_USER_CCL_LUTIN_0
    case 0: return EVENT_ID_USER_CCL_LUTIN_0;
#endif
#ifdef EVENT_ID_USER_CCL_LUTIN_1
    case 1: return EVENT_ID_USER_CCL_LUTIN_1;
#endif
#ifdef EVENT_ID_USER_CCL_LUTIN_2
    case 2: return EVENT_ID_USER_CCL_LUTIN_2;
#endif
#ifdef EVENT_ID_USER_CCL_LUTIN_3
    case 3: return EVENT_ID_USER_CCL_LUTIN_3;
#endif
    default: return 0;
    }
}

/// Whether this device header knows INSEL's two N-variant-only codes
/// (37.8.3): ALT2TC = 0xA and ASYNCEVENT = 0xB. Both are absent from
/// the E/G/J headers, which is the one question no template can ask -
/// and the answer the driver's refusals are built on.
constexpr bool ccl_has_alt2_tc() {
#ifdef CCL_LUTCTRL_INSEL0_ALT2TC_Val
    return true;
#else
    return false;
#endif
}

constexpr bool ccl_has_async_event() {
#ifdef CCL_LUTCTRL_INSEL0_ASYNCEVENT_Val
    return true;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------
// pad -> CCL input line, and pad -> CCL output
//
// UNLIKE THE EIC AND TC MAPS, THE VALUE IS IN THE SYMBOL'S NAME and not
// in what it expands to: `PIN_PA04I_CCL_IN0` holds the PAD's number (4),
// where `PIN_PA04A_EIC_EXTINT_NUM` holds the LINE's. So each entry below
// states its index literally and the `#ifdef` answers the only question
// left - whether this package bonds that pad to that function at all.
// Which is still a probe and still cannot go stale: a package that drops
// a pad drops its symbol, and the entry disappears with it.
//
// The input line numbering runs across the whole block: IN[3k..3k+2] are
// LUT k's inputs 0, 1 and 2 (37.4's IN[11:0] against four LUTs of three).

#define BRIO_CCL_IN(letter, number, index) \
    case (static_cast<int>(letter) - 'A') * 32 + (number): return (index);

/// This pad's CCL input line, or -1 if this device does not bond one.
constexpr int ccl_in_line(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA04I_CCL_IN0
    BRIO_CCL_IN('A', 4, 0)
#endif
#ifdef PIN_PA05I_CCL_IN1
    BRIO_CCL_IN('A', 5, 1)
#endif
#ifdef PIN_PA06I_CCL_IN2
    BRIO_CCL_IN('A', 6, 2)
#endif
#ifdef PIN_PA08I_CCL_IN3
    BRIO_CCL_IN('A', 8, 3)
#endif
#ifdef PIN_PA09I_CCL_IN4
    BRIO_CCL_IN('A', 9, 4)
#endif
#ifdef PIN_PA10I_CCL_IN5
    BRIO_CCL_IN('A', 10, 5)
#endif
#ifdef PIN_PA16I_CCL_IN0
    BRIO_CCL_IN('A', 16, 0)
#endif
#ifdef PIN_PA17I_CCL_IN1
    BRIO_CCL_IN('A', 17, 1)
#endif
#ifdef PIN_PA18I_CCL_IN2
    BRIO_CCL_IN('A', 18, 2)
#endif
#ifdef PIN_PA22I_CCL_IN6
    BRIO_CCL_IN('A', 22, 6)
#endif
#ifdef PIN_PA23I_CCL_IN7
    BRIO_CCL_IN('A', 23, 7)
#endif
#ifdef PIN_PA24I_CCL_IN8
    BRIO_CCL_IN('A', 24, 8)
#endif
#ifdef PIN_PA30I_CCL_IN3
    BRIO_CCL_IN('A', 30, 3)
#endif
#ifdef PIN_PB00I_CCL_IN1
    BRIO_CCL_IN('B', 0, 1)
#endif
#ifdef PIN_PB01I_CCL_IN2
    BRIO_CCL_IN('B', 1, 2)
#endif
#ifdef PIN_PB06I_CCL_IN6
    BRIO_CCL_IN('B', 6, 6)
#endif
#ifdef PIN_PB07I_CCL_IN7
    BRIO_CCL_IN('B', 7, 7)
#endif
#ifdef PIN_PB08I_CCL_IN8
    BRIO_CCL_IN('B', 8, 8)
#endif
#ifdef PIN_PB10I_CCL_IN5
    BRIO_CCL_IN('B', 10, 5)
#endif
#ifdef PIN_PB14I_CCL_IN9
    BRIO_CCL_IN('B', 14, 9)
#endif
#ifdef PIN_PB15I_CCL_IN10
    BRIO_CCL_IN('B', 15, 10)
#endif
#ifdef PIN_PB16I_CCL_IN11
    BRIO_CCL_IN('B', 16, 11)
#endif
#ifdef PIN_PB22I_CCL_IN0
    BRIO_CCL_IN('B', 22, 0)
#endif
    default: return -1;
    }
}

/// This pad's CCL output - which IS the LUT's own number (OUT[3:0]) -
/// or -1 if this device does not bond one.
constexpr int ccl_out_lut(char port, uint8_t pin) {
    switch ((static_cast<int>(port) - 'A') * 32 + static_cast<int>(pin)) {
#ifdef PIN_PA07I_CCL_OUT0
    BRIO_CCL_IN('A', 7, 0)
#endif
#ifdef PIN_PA11I_CCL_OUT1
    BRIO_CCL_IN('A', 11, 1)
#endif
#ifdef PIN_PA19I_CCL_OUT0
    BRIO_CCL_IN('A', 19, 0)
#endif
#ifdef PIN_PA25I_CCL_OUT2
    BRIO_CCL_IN('A', 25, 2)
#endif
#ifdef PIN_PA31I_CCL_OUT1
    BRIO_CCL_IN('A', 31, 1)
#endif
#ifdef PIN_PB02I_CCL_OUT0
    BRIO_CCL_IN('B', 2, 0)
#endif
#ifdef PIN_PB09I_CCL_OUT2
    BRIO_CCL_IN('B', 9, 2)
#endif
#ifdef PIN_PB11I_CCL_OUT1
    BRIO_CCL_IN('B', 11, 1)
#endif
#ifdef PIN_PB17I_CCL_OUT3
    BRIO_CCL_IN('B', 17, 3)
#endif
#ifdef PIN_PB23I_CCL_OUT0
    BRIO_CCL_IN('B', 23, 0)
#endif
    default: return -1;
    }
}

#undef BRIO_CCL_IN

/// Whether a LUT has ANY input pad on this package, and whether it has
/// its output pad. On the E and the G, LUT3 has neither: it exists in
/// silicon and is reachable only through events, a link or a sequencer.
constexpr bool ccl_lut_has_input_pad(uint8_t lut) {
    for (int port = 0; port < 2; ++port) {
        for (uint8_t pin = 0; pin < 32; ++pin) {
            const int line = ccl_in_line(static_cast<char>('A' + port), pin);
            if (line >= 0 && static_cast<uint8_t>(line / 3) == lut) {
                return true;
            }
        }
    }
    return false;
}

constexpr bool ccl_lut_has_output_pad(uint8_t lut) {
    for (int port = 0; port < 2; ++port) {
        for (uint8_t pin = 0; pin < 32; ++pin) {
            if (ccl_out_lut(static_cast<char>('A' + port), pin) ==
                static_cast<int>(lut)) {
                return true;
            }
        }
    }
    return false;
}

template <char L, uint8_t N>
constexpr bool ccl_in_exists = ccl_in_line(L, N) >= 0;
template <char L, uint8_t N>
constexpr bool ccl_out_exists = ccl_out_lut(L, N) >= 0;

} // namespace brio
