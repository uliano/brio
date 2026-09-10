/*
 * opa.hpp
 *
 * The operational amplifier of the CH32V00x (RM ch. 17): one OPA with a
 * selectable positive input (four pads), a negative input that is a pad
 * or an internal gain of 4, 8, 16 or 32 (the PGA, with a 192 kOhm
 * feedback the block switches in), a differential PGA whose negative
 * side is PA4, a bias reference of VDD/2 or VDD/4 for the PGA, a
 * high-speed mode, an output that is a pad (PD4 or PA5) and ALWAYS the
 * ADC's channel 9 - and the comparator CMP2 that reads the OPA's
 * output against that same bias. The chapter's second comparator, CMP1
 * with its own pads, exists on the CH32V007 alone (17's opening
 * paragraph), so this file spells CMP2 and the polling and leaves CMP1
 * for the part that has it.
 *
 * THE LOCKS. OPA_CTLR1.OPA_LOCK is SET at reset: nothing in the register
 * takes a write until the two keys go into OPA_KEY in order (17.3.5,
 * the flash's own pair 0x45670123 / 0xCDEF89AB), and OPA_LOCK written
 * one locks it again until the next system reset. CMP_KEY and POLL_KEY
 * are the same arrangement for OPA_CTLR2 and the polling fields.
 *
 * WHAT THE OPA REACHES WITH NO PAD: its output is the ADC's channel 9
 * whatever MODE1 says (measured: the single-ended PGA at a gain of 4
 * reads 2 counts with its input pulled down and 4095 pulled up). THE
 * DIFFERENTIAL PGA'S INPUTS ARE NOT HIGH-IMPEDANCE: figure 17-1 draws
 * 3.2 kOhm into its network, and a pad's own pull cannot hold such an
 * input at a rail - both inputs pulled up read near zero on channel 9,
 * both pulled down near 0.77 VDD, neither the bias reference - so the
 * bias, the gains and the differential transfer want a SOURCE.
 *
 * CMP2 ON THIS PART: CMP_KEY lifts CMP_LOCK, and CMP_EN2 does not take
 * a write (measured) - the comparators are the CH32V007's, as 17's
 * opening paragraph says of the CMP module; the verbs stay for the
 * part that has them and answer false here.
 *
 * NOT COVERED YET: the front-end POLLING (CFGR1/CFGR2: three P-side
 * channels sampled in turn on a timer trigger, each into the ADC, with
 * a per-channel window and a reset on a fault) - the shape of a task
 * this stratum has no user for; CMP2's output as TIM1's break source
 * (BKIN_CFG) and its filter - the break unit's, born with a
 * measurement; the OPA on an external feedback (a resistor across the
 * pads) - a wire.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pin.hpp"

namespace brio {

struct OpaRegs {
    volatile uint32_t CFGR1;     ///< 0x00 the polling
    volatile uint32_t CTLR1;     ///< 0x04 the OPA
    volatile uint32_t CFGR2;     ///< 0x08 the polling's second half
    volatile uint32_t CTLR2;     ///< 0x0c the comparators
    volatile uint32_t OPA_KEY;   ///< 0x10 write-only
    volatile uint32_t CMP_KEY;   ///< 0x14
    volatile uint32_t POLL_KEY;  ///< 0x18
};

inline constexpr uint32_t opa_base = 0x40024000UL;
inline OpaRegs* opa() { return reinterpret_cast<OpaRegs*>(opa_base); }

inline constexpr uint32_t opa_key1 = 0x45670123UL;
inline constexpr uint32_t opa_key2 = 0xCDEF89ABUL;

// CTLR1 (17.3.2)
inline constexpr uint32_t opa_en1      = 1UL << 0;
inline constexpr uint32_t opa_mode_mask = 3UL << 1;
inline constexpr uint32_t opa_psel_mask = 3UL << 4;
inline constexpr uint32_t opa_nsel_mask = 7UL << 8;
inline constexpr uint32_t opa_fb_en1   = 1UL << 11;
inline constexpr uint32_t opa_pgadif   = 1UL << 12;
inline constexpr uint32_t opa_vben     = 1UL << 16;
inline constexpr uint32_t opa_vbsel    = 1UL << 17;
inline constexpr uint32_t opa_vbcmpsel_mask = 3UL << 18;
inline constexpr uint32_t opa_hs1      = 1UL << 20;
inline constexpr uint32_t opa_lock     = 1UL << 31;
// CTLR2 (17.3.4)
inline constexpr uint32_t opa_cmp_en1      = 1UL << 0;
inline constexpr uint32_t opa_cmp_hyen1    = 1UL << 7;
inline constexpr uint32_t opa_cmp_rmid1    = 1UL << 8;
inline constexpr uint32_t opa_cmp_en2      = 1UL << 16;
inline constexpr uint32_t opa_cmp_filt_en  = 1UL << 24;
inline constexpr uint32_t opa_cmp_filt_sel = 1UL << 25;
inline constexpr uint32_t opa_cmp_bkin_mask = 3UL << 26;
inline constexpr uint32_t opa_cmp_lock     = 1UL << 31;

/// PSEL1: the positive input's pad.
enum class OpaPositive : uint8_t { pa2 = 0, pd7 = 1, pd3 = 2, pd1 = 3 };

/// NSEL1: the negative input - a pad, or the internal gain of a PGA.
enum class OpaNegative : uint8_t {
    pa1 = 0,        ///< OPA_CHN0
    pd0 = 1,        ///< OPA_CHN1
    gain4 = 3,      ///< PGA, no negative pad
    gain8 = 4,
    gain16 = 5,
    gain32 = 6,     ///< not with the differential PGA (17.3.2's note)
    off = 7,
};

constexpr bool opa_negative_is_gain(OpaNegative n) {
    return n == OpaNegative::gain4 || n == OpaNegative::gain8 || n == OpaNegative::gain16 ||
           n == OpaNegative::gain32;
}
constexpr uint8_t opa_gain_of(OpaNegative n) {
    switch (n) {
        case OpaNegative::gain4: return 4;
        case OpaNegative::gain8: return 8;
        case OpaNegative::gain16: return 16;
        case OpaNegative::gain32: return 32;
        default: return 1;
    }
}

/// MODE1: where the output goes beside the ADC's channel 9 and CMP2.
enum class OpaOutput : uint8_t { pd4 = 0, pa5 = 1, internal = 2 };

/// The PGA's bias reference (VBSEL, under VBEN).
enum class OpaBias : uint8_t { none, vdd_over_2, vdd_over_4 };

/// VBCMPSEL: CMP2's negative reference (table 17-3), or off.
enum class OpaCmp2Reference : uint8_t { code0 = 0, code1 = 1, code2 = 2, off = 3 };

struct OpaConfig {
    OpaPositive positive = OpaPositive::pa2;
    OpaNegative negative = OpaNegative::gain4;
    OpaOutput output = OpaOutput::internal;
    bool feedback = true;            ///< FB_EN1: MUST be set in the PGA modes (17.3.2's note)
    bool differential = false;       ///< PGADIF: the negative side is PA4
    OpaBias bias = OpaBias::none;    ///< VBEN/VBSEL
    bool high_speed = false;         ///< OPA_HS1, 40 V/us
    OpaCmp2Reference cmp2_reference = OpaCmp2Reference::off;
};

/// The refusals the chapter states: a PGA gain wants the feedback
/// switched in, the differential PGA has no gain 32, a bias reference
/// only under a PGA gain.
constexpr bool opa_config_valid(const OpaConfig& c) {
    if (opa_negative_is_gain(c.negative) && !c.feedback) {
        return false;
    }
    if (c.differential && (c.negative == OpaNegative::gain32 || !opa_negative_is_gain(c.negative))) {
        return false;
    }
    if (c.bias != OpaBias::none && !opa_negative_is_gain(c.negative)) {
        return false;
    }
    return true;
}

constexpr uint32_t opa_ctlr1_of(const OpaConfig& c) {
    uint32_t v = 0;
    v |= static_cast<uint32_t>(c.output) << 1;
    v |= static_cast<uint32_t>(c.positive) << 4;
    v |= static_cast<uint32_t>(c.negative) << 8;
    if (c.feedback) { v |= opa_fb_en1; }
    if (c.differential) { v |= opa_pgadif; }
    if (c.bias != OpaBias::none) {
        v |= opa_vben;
        if (c.bias == OpaBias::vdd_over_4) { v |= opa_vbsel; }
    }
    v |= static_cast<uint32_t>(c.cmp2_reference) << 18;
    if (c.high_speed) { v |= opa_hs1; }
    return v;
}

/// The pads of each input and output, for a program that claims them.
constexpr Pad opa_positive_pad(OpaPositive p) {
    switch (p) {
        case OpaPositive::pa2: return {'A', 2};
        case OpaPositive::pd7: return {'D', 7};
        case OpaPositive::pd3: return {'D', 3};
        default: return {'D', 1};
    }
}
inline constexpr Pad opa_differential_negative_pad{'A', 4};
constexpr Pad opa_negative_pad(OpaNegative n) {
    return n == OpaNegative::pa1 ? Pad{'A', 1} : n == OpaNegative::pd0 ? Pad{'D', 0} : Pad{};
}
constexpr Pad opa_output_pad(OpaOutput o) {
    return o == OpaOutput::pd4 ? Pad{'D', 4} : o == OpaOutput::pa5 ? Pad{'A', 5} : Pad{};
}

/**
 * The OPA, a monostate. Every configuring verb needs the block unlocked
 * (unlock() first, once per reset); the ADC reads the output on its
 * channel 9 whatever the output selection.
 *
 *   Opa::unlock();
 *   Opa::configure({.positive = OpaPositive::pa2, .negative = OpaNegative::gain8});
 *   Opa::enable(true);
 */
struct Opa {
    Opa() = delete;

    static OpaRegs& regs() { return *opa(); }
    /// The ADC channel the output is always on (17's overview).
    static constexpr uint8_t adc_channel = 9;

    /// The key pair, in order. Nothing takes before this.
    static void unlock() {
        regs().OPA_KEY = opa_key1;
        regs().OPA_KEY = opa_key2;
    }
    /// One-way until the next system reset.
    static void lock() { regs().CTLR1 |= opa_lock; }
    static bool locked() { return (regs().CTLR1 & opa_lock) != 0u; }

    /// The whole configuration, the enable left as it is. False for a
    /// config the chapter refuses, or with the block locked.
    static bool configure(const OpaConfig& c) {
        if (!opa_config_valid(c) || locked()) {
            return false;
        }
        regs().CTLR1 = (regs().CTLR1 & opa_en1) | opa_ctlr1_of(c);
        return true;
    }
    static void enable(bool on) {
        regs().CTLR1 = on ? (regs().CTLR1 | opa_en1) : (regs().CTLR1 & ~opa_en1);
    }
    static bool enabled() { return (regs().CTLR1 & opa_en1) != 0u; }
    static uint32_t ctlr1() { return regs().CTLR1; }

    // ---- CMP2: the OPA's output against the bias reference ------------------

    static void cmp_unlock() {
        regs().CMP_KEY = opa_key1;
        regs().CMP_KEY = opa_key2;
    }
    static void cmp_lock() { regs().CTLR2 |= opa_cmp_lock; }
    static bool cmp_locked() { return (regs().CTLR2 & opa_cmp_lock) != 0u; }
    /// True when the bit TOOK - which on the CH32V006 it does not (the
    /// file header).
    static bool cmp2_enable(bool on) {
        if (cmp_locked()) {
            return false;
        }
        regs().CTLR2 = on ? (regs().CTLR2 | opa_cmp_en2) : (regs().CTLR2 & ~opa_cmp_en2);
        return cmp2_enabled() == on;
    }
    static bool cmp2_enabled() { return (regs().CTLR2 & opa_cmp_en2) != 0u; }
};

// The chapter's refusals, pinned.
static_assert(opa_config_valid(OpaConfig{}));
static_assert(!opa_config_valid(OpaConfig{.negative = OpaNegative::gain8, .feedback = false}));
static_assert(!opa_config_valid(OpaConfig{.negative = OpaNegative::gain32, .differential = true}));
static_assert(!opa_config_valid(OpaConfig{.negative = OpaNegative::pa1, .bias = OpaBias::vdd_over_2}));
static_assert(opa_config_valid(OpaConfig{.negative = OpaNegative::gain16, .differential = true, .bias = OpaBias::vdd_over_4}));
static_assert(opa_ctlr1_of(OpaConfig{.positive = OpaPositive::pd3, .negative = OpaNegative::gain8,
                                     .output = OpaOutput::pd4, .bias = OpaBias::vdd_over_4}) ==
              ((2UL << 4) | (4UL << 8) | opa_fb_en1 | opa_vben | opa_vbsel | (3UL << 18)));
static_assert(opa_gain_of(OpaNegative::gain32) == 32u && opa_gain_of(OpaNegative::pa1) == 1u);
static_assert(opa_positive_pad(OpaPositive::pd7) == Pad{'D', 7} && opa_output_pad(OpaOutput::internal) == Pad{});

} // namespace brio
