/*
 * opa.hpp
 *
 * The operational amplifiers of the CH32V203 (RM ch. 30): TWO of them,
 * each with a positive input that is one of two pads, a negative input
 * that is one of two pads, and an output that is one of two pads. That
 * is the whole chapter - four bits an amplifier in ONE register, no
 * key, no lock, no interrupt, no event, no DMA, and nothing that is
 * refused while it runs.
 *
 * WHAT IT IS NOT, WHICH MATTERS MORE THAN WHAT IT IS. There is no
 * internal feedback network here and no gain: the CH32V00x's block
 * switches a 192 kOhm resistor in and offers gains of 4 to 32, and this
 * one offers neither, so the amplifier as WCH ships it on this family
 * is an open-loop stage. The datasheet calls it "operational
 * amplifier/comparator" and rates it at 136 dB of open-loop gain, which
 * is the same statement said twice: with a wire from an output pad back
 * to a negative input pad it is a follower or a divider, and with no
 * wire at all it is a comparator whose output saturates to a rail.
 * THE GAIN A PROGRAM CAN REACH WITH NO WIRE IS THE ADC's (CTLR1.PGA,
 * ch32v203/adc.hpp), which sits in front of the converter and not here.
 *
 * WHERE THE OUTPUT GOES. Every OPA output pad is also an ADC input pad
 * - OPA1 reaches PA3 (channel 3) or PB1 (channel 9), OPA2 reaches PA2
 * (channel 2) or PA4 (channel 4) - which is what 30.2 means by "the
 * output pin can select general-purpose I/O or the ADC sampling
 * channel": there is no internal route, the pad is the route, and the
 * converter reads the amplifier by converting that pad's own channel.
 * `OpaOut<n, which>::adc_channel` is that number.
 *
 * WHICH AMPLIFIERS A PART HAS is `device::has_opa(n)` (datasheet table
 * 2-1): both on every part but the CH32V203F6, whose twenty pins bond
 * neither of OPA1's positive inputs and which the table therefore
 * counts as having one. `Opa<1>` does not compile there. WHICH PADS a
 * package bonds is `Pin`'s own refusal, as everywhere in this stratum.
 *
 * OPA3 AND OPA4 ARE NOT OURS: bits 8 to 15 of the register carry them
 * and every one of those bit descriptions names CH32F20x_D8,
 * CH32F20x_D8C, CH32V30x_D8, CH32V30x_D8C and CH32V31x_D8C - no
 * CH32V20x - so this file spells two amplifiers and the register's
 * upper half is not reached.
 *
 * THE REGISTER LIVES IN THE EXTEN BLOCK'S WINDOW, at 0x40023804,
 * between EXTEN_CTR and EXTEN_CTR2 (which is why device.hpp's ExtenRegs
 * shows a reserved word there). Like EXTEN it has no clock gate in RCC
 * and no reset line: a program writes it and it is written.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/device.hpp"
#include "ch32v203/pin.hpp"

namespace brio {

// =============================================================================
// The register (RM table 30-1)
// =============================================================================

struct OpaRegs {
    volatile uint32_t CTLR;   ///< 0x00 - four bits per amplifier
};

inline constexpr uint32_t opa_base = hb_base + 0x3804;
inline OpaRegs* opa() { return reinterpret_cast<OpaRegs*>(opa_base); }

/// The four bits of amplifier `n`, at 4 x (n - 1) (30.3.1).
constexpr uint32_t opa_shift_for(uint8_t n) { return 4u * (n - 1u); }
inline constexpr uint32_t opa_en_bit   = 1UL << 0;   ///< ENx
inline constexpr uint32_t opa_mode_bit = 1UL << 1;   ///< MODEx: which output pad
inline constexpr uint32_t opa_nsel_bit = 1UL << 2;   ///< NSELx: which negative pad
inline constexpr uint32_t opa_psel_bit = 1UL << 3;   ///< PSELx: which positive pad
inline constexpr uint32_t opa_field_mask = 0xFUL;

// =============================================================================
// The vocabulary
// =============================================================================

/// The six pads an amplifier can reach, named as the datasheet's pin
/// tables name them (OPAx_CH0P, OPAx_CH1N, OPAx_OUT1 and their kin).
/// One enum for all three roles, because a configuration that put an
/// output code in a positive input's place would otherwise be spelled
/// and only then refused.
enum class OpaPin : uint8_t {
    positive0,   ///< CHP0 - PSEL = 0
    positive1,   ///< CHP1 - PSEL = 1
    negative0,   ///< CHN0 - NSEL = 0
    negative1,   ///< CHN1 - NSEL = 1
    out0,        ///< OUT0 - MODE = 0
    out1,        ///< OUT1 - MODE = 1
};

constexpr bool opa_is_positive(OpaPin p) {
    return p == OpaPin::positive0 || p == OpaPin::positive1;
}
constexpr bool opa_is_negative(OpaPin p) {
    return p == OpaPin::negative0 || p == OpaPin::negative1;
}
constexpr bool opa_is_output(OpaPin p) { return p == OpaPin::out0 || p == OpaPin::out1; }
/// The bit value the register takes for that pad, within its own role.
constexpr uint8_t opa_code(OpaPin p) {
    return (p == OpaPin::positive1 || p == OpaPin::negative1 || p == OpaPin::out1) ? 1u : 0u;
}

/**
 * The pad map (datasheet 3.2's pin tables, the same on every package of
 * the series; what a package BONDS is the part's). An invalid pad for
 * an amplifier this family has not got.
 */
constexpr Pad opa_pad(uint8_t n, OpaPin which) {
    if (n == 1u) {
        switch (which) {
            case OpaPin::positive0: return Pad{'B', 15};
            case OpaPin::positive1: return Pad{'B', 0};
            case OpaPin::negative0: return Pad{'B', 11};
            case OpaPin::negative1: return Pad{'A', 6};
            case OpaPin::out0:      return Pad{'A', 3};
            case OpaPin::out1:      return Pad{'B', 1};
        }
    } else if (n == 2u) {
        switch (which) {
            case OpaPin::positive0: return Pad{'B', 14};
            case OpaPin::positive1: return Pad{'A', 7};
            case OpaPin::negative0: return Pad{'B', 10};
            case OpaPin::negative1: return Pad{'A', 5};
            case OpaPin::out0:      return Pad{'A', 2};
            case OpaPin::out1:      return Pad{'A', 4};
        }
    }
    return Pad{};
}

/// The ADC channel an output pad carries (ch32v203/adc.hpp's own map,
/// repeated here as the two numbers this chapter needs rather than as a
/// dependency: the OPA does not otherwise know the converter).
constexpr uint8_t opa_output_channel(uint8_t n, OpaPin which) {
    const Pad p = opa_pad(n, which);
    if (!opa_is_output(which) || !p.valid()) {
        return 0xFF;
    }
    return p.port == 'A' ? p.pin : /* PB1 */ uint8_t{9};
}

/// What a configuration says, and what it must not say.
struct OpaConfig {
    OpaPin positive = OpaPin::positive0;   ///< PSELx
    OpaPin negative = OpaPin::negative0;   ///< NSELx
    OpaPin output = OpaPin::out0;          ///< MODEx
};

constexpr bool opa_config_valid(const OpaConfig& c) {
    return opa_is_positive(c.positive) && opa_is_negative(c.negative) && opa_is_output(c.output);
}

/// The four bits a configuration makes, the enable left out.
constexpr uint32_t opa_config_bits(const OpaConfig& c) {
    uint32_t v = 0;
    if (opa_code(c.output) != 0u) { v |= opa_mode_bit; }
    if (opa_code(c.negative) != 0u) { v |= opa_nsel_bit; }
    if (opa_code(c.positive) != 0u) { v |= opa_psel_bit; }
    return v;
}

/**
 * OpaIn<n, which>: one of an amplifier's four input pads, claimed as an
 * analog pad. Refused at compile time for an output code, for an
 * amplifier this part has not got, and - through `Pin` itself - for a
 * pad this package does not bond.
 */
template <uint8_t n, OpaPin which>
struct OpaIn {
    static_assert(device::has_opa(n),
                  "brio OpaIn: this part has no such amplifier (datasheet table 2-1: two on "
                  "every part but the twenty-pin one, which bonds neither of OPA1's positive "
                  "inputs and counts one)");
    static_assert(opa_is_positive(which) || opa_is_negative(which),
                  "brio OpaIn: an input pad is positive0, positive1, negative0 or negative1 - "
                  "an output is OpaOut's");
    static constexpr uint8_t amplifier = n;
    static constexpr Pad pad = opa_pad(n, which);
    using pin = Pin<pad.port, pad.pin>;

    /// The analog mode of RM 10.2.7: the input driver off, and the pull
    /// gone with it. An input the program wants at a known level is
    /// driven or pulled by the PORT instead, and this is not called.
    static void claim() { pin::analog(); }
};

/**
 * OpaOut<n, which>: one of an amplifier's two output pads, with the ADC
 * channel it carries. `claim()` puts the pad in analog mode, which is
 * what the converter reads it through.
 */
template <uint8_t n, OpaPin which>
struct OpaOut {
    static_assert(device::has_opa(n),
                  "brio OpaOut: this part has no such amplifier (datasheet table 2-1)");
    static_assert(opa_is_output(which), "brio OpaOut: an output pad is out0 or out1");
    static constexpr uint8_t amplifier = n;
    static constexpr Pad pad = opa_pad(n, which);
    static constexpr uint8_t adc_channel = opa_output_channel(n, which);
    using pin = Pin<pad.port, pad.pin>;

    static void claim() { pin::analog(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Opa<n>: one amplifier.
 *
 *   using Out = brio::OpaOut<2, brio::OpaPin::out0>;       // PA2, channel 2
 *   brio::OpaIn<2, brio::OpaPin::positive1>::pin::output_high();
 *   brio::OpaIn<2, brio::OpaPin::negative1>::pin::output_low();
 *   Out::claim();
 *   brio::Opa<2>::configure({.positive = brio::OpaPin::positive1,
 *                            .negative = brio::OpaPin::negative1,
 *                            .output = brio::OpaPin::out0});
 *   brio::Opa<2>::enable(true);
 *   const uint16_t counts = brio::Adc<1>::read();           // with channel 2 selected
 *
 * The pads are the caller's: this class writes four bits and nothing
 * else, because four bits is the whole chapter.
 */
template <uint8_t n>
class Opa {
public:
    Opa() = delete;

    static_assert(n == 1u || n == 2u,
                  "brio Opa: this family has OPA1 and OPA2 - the register's upper half is "
                  "OPA3 and OPA4, and every one of those bits names another device class "
                  "(RM 30.3.1)");
    static_assert(device::has_opa(n),
                  "brio Opa: this part has no such amplifier (datasheet table 2-1: the "
                  "twenty-pin part bonds neither of OPA1's positive inputs and counts one)");

    static constexpr uint8_t instance = n;
    static constexpr uint32_t shift = opa_shift_for(n);

    static OpaRegs& regs() { return *opa(); }

    /// The four bits this amplifier owns, as they read.
    static uint8_t field() {
        return static_cast<uint8_t>((regs().CTLR >> shift) & opa_field_mask);
    }

    /// The three selections, the enable left as it is. False for a
    /// configuration that put a pad in the wrong role.
    static bool configure(const OpaConfig& c) {
        if (!opa_config_valid(c)) {
            return false;
        }
        const uint32_t keep = regs().CTLR & ~(opa_field_mask << shift);
        const uint32_t bits = opa_config_bits(c) | (enabled() ? opa_en_bit : 0u);
        regs().CTLR = keep | (bits << shift);
        return true;
    }

    /// What the register says this amplifier is wired to.
    static OpaConfig configuration() {
        const uint8_t f = field();
        OpaConfig c{};
        c.positive = (f & opa_psel_bit) != 0u ? OpaPin::positive1 : OpaPin::positive0;
        c.negative = (f & opa_nsel_bit) != 0u ? OpaPin::negative1 : OpaPin::negative0;
        c.output = (f & opa_mode_bit) != 0u ? OpaPin::out1 : OpaPin::out0;
        return c;
    }

    static void enable(bool on) {
        const uint32_t bit = opa_en_bit << shift;
        regs().CTLR = on ? (regs().CTLR | bit) : (regs().CTLR & ~bit);
    }
    static bool enabled() { return (regs().CTLR & (opa_en_bit << shift)) != 0u; }

    /// The selections without a whole configuration, for a program that
    /// switches one input while the amplifier runs.
    static bool positive(OpaPin p) {
        if (!opa_is_positive(p)) {
            return false;
        }
        return set_bit(opa_psel_bit, opa_code(p) != 0u);
    }
    static bool negative(OpaPin p) {
        if (!opa_is_negative(p)) {
            return false;
        }
        return set_bit(opa_nsel_bit, opa_code(p) != 0u);
    }
    static bool output(OpaPin p) {
        if (!opa_is_output(p)) {
            return false;
        }
        return set_bit(opa_mode_bit, opa_code(p) != 0u);
    }

    /// The pad each selection names, at run time.
    static Pad pad(OpaPin which) { return opa_pad(n, which); }
    /// The ADC channel the output currently selected reaches.
    static uint8_t adc_channel() { return opa_output_channel(n, configuration().output); }

    /// Back to the reset state, this amplifier's bits alone.
    static void release() { regs().CTLR = regs().CTLR & ~(opa_field_mask << shift); }

private:
    static bool set_bit(uint32_t bit, bool on) {
        const uint32_t b = bit << shift;
        regs().CTLR = on ? (regs().CTLR | b) : (regs().CTLR & ~b);
        return true;
    }
};

// =============================================================================
// The chapter, pinned at compile time
// =============================================================================

static_assert(sizeof(OpaRegs) == 4);
static_assert(opa_base == 0x40023804UL);
static_assert(opa_shift_for(1) == 0u && opa_shift_for(2) == 4u);
static_assert(opa_pad(1, OpaPin::positive0) == Pad{'B', 15});
static_assert(opa_pad(1, OpaPin::positive1) == Pad{'B', 0});
static_assert(opa_pad(1, OpaPin::negative0) == Pad{'B', 11});
static_assert(opa_pad(1, OpaPin::negative1) == Pad{'A', 6});
static_assert(opa_pad(1, OpaPin::out0) == Pad{'A', 3});
static_assert(opa_pad(1, OpaPin::out1) == Pad{'B', 1});
static_assert(opa_pad(2, OpaPin::positive0) == Pad{'B', 14});
static_assert(opa_pad(2, OpaPin::positive1) == Pad{'A', 7});
static_assert(opa_pad(2, OpaPin::negative0) == Pad{'B', 10});
static_assert(opa_pad(2, OpaPin::negative1) == Pad{'A', 5});
static_assert(opa_pad(2, OpaPin::out0) == Pad{'A', 2});
static_assert(opa_pad(2, OpaPin::out1) == Pad{'A', 4});
static_assert(!opa_pad(3, OpaPin::out0).valid());
// Every output pad is an ADC input pad, which is the whole of this
// chapter's route to the converter.
static_assert(opa_output_channel(1, OpaPin::out0) == 3u);
static_assert(opa_output_channel(1, OpaPin::out1) == 9u);
static_assert(opa_output_channel(2, OpaPin::out0) == 2u);
static_assert(opa_output_channel(2, OpaPin::out1) == 4u);
static_assert(opa_output_channel(1, OpaPin::positive0) == 0xFFu);
static_assert(opa_config_valid(OpaConfig{}));
static_assert(!opa_config_valid(OpaConfig{.positive = OpaPin::out0}));
static_assert(!opa_config_valid(OpaConfig{.negative = OpaPin::positive0}));
static_assert(!opa_config_valid(OpaConfig{.output = OpaPin::negative1}));
static_assert(opa_config_bits(OpaConfig{}) == 0u);
static_assert(opa_config_bits(OpaConfig{.positive = OpaPin::positive1,
                                        .negative = OpaPin::negative1,
                                        .output = OpaPin::out1}) == 0xEu);

} // namespace brio
