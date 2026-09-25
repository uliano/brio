/*
 * opa.hpp
 *
 * The operational amplifiers of the CH32V203 and the CH32V303 (RM ch. 30):
 * up to FOUR of them, each with a positive input that is one of two pads, a
 * negative input that is one of two pads, and an output that is one of two
 * pads. That is the whole chapter - four bits an amplifier in ONE register,
 * no key, no lock, no interrupt, no event, no DMA, and nothing that is
 * refused while it runs - plus, on the CH32V303, one high-speed bit an
 * amplifier in another register.
 *
 * WHAT IT IS NOT, WHICH MATTERS MORE THAN WHAT IT IS. There is no
 * internal feedback network here and no gain: the CH32V00x's block
 * switches a 192 kOhm resistor in and offers gains of 4 to 32, and this
 * one offers neither, so the amplifier as WCH ships it on this family
 * is an open-loop stage. The datasheets call it "operational
 * amplifier/comparator" and rate it at 136 dB of open-loop gain, which
 * is the same statement said twice: with a wire from an output pad back
 * to a negative input pad it is a follower or a divider, and with no
 * wire at all it is a comparator whose output saturates to a rail.
 * THE GAIN A PROGRAM CAN REACH WITH NO WIRE IS THE ADC's (CTLR1.PGA,
 * ch32vx03/adc.hpp), which sits in front of the converter and not here.
 *
 * HOW MANY A PART HAS, AND WHICH, is `device::has_opa(n)` (the datasheets'
 * tables 2-1 and 2-1-1): two on every CH32V203 but the CH32V203F6, whose
 * twenty pins bond neither of OPA1's positive inputs and which its table
 * therefore counts as having one (OPA2); four on every CH32V303. The
 * register's upper half, OPA3 and OPA4, carries the note "Applied for
 * CH32F20x_D8, CH32F20x_D8C, CH32V30x_D8, CH32V30x_D8C and CH32V31x_D8C"
 * on every one of its eight fields (30.3.1) - the CH32V303's class and no
 * CH32V203's - and nothing else in the register is class-keyed. WHICH
 * PADS a package bonds is `Pin`'s own refusal, as everywhere in this
 * stratum.
 *
 * THE PADS ARE THE CLASS'S (the datasheets' pin tables and the CH32V303's
 * table 3-4). The first output of OPA1 and OPA2 and all four inputs are the
 * same pads on both series; the SECOND output is not - PB1 and PA4 on the
 * CH32V203, PE15 and PE14 on the CH32V303, whose OPA3 and OPA4 put theirs on
 * PE7 and PE8. `opa_pad()` reads the class.
 *
 * WHERE THE OUTPUT GOES. There is no internal route: 30.2's "the output
 * pin can select general-purpose I/O or the ADC sampling channel" means
 * the PAD, and the converter reads the amplifier by converting that pad's
 * own channel - OUT0 is PA3, PA2, PA1 or PA0 (ADC channels 3, 2, 1 and 0)
 * for OPA1..OPA4 on every part. On the CH32V203 every output pad is an ADC
 * input (OUT1 is PB1, channel 9, and PA4, channel 4); on the CH32V303 the
 * OUT1 pads of port E are NOT - `OpaOut<n, out1>::adc_channel` says 0xFF
 * there - and the CH32V303's datasheet (table 4-46, note 3) limits their
 * swing to 0..2 V at a VDDA of 3.3 V.
 *
 * THE REGISTER LIVES IN THE EXTEN BLOCK'S WINDOW, at 0x40023804,
 * between EXTEN_CTR and EXTEN_CTR2 (which is why device.hpp's ExtenRegs
 * shows a reserved word there). Like EXTEN it has no clock gate in RCC
 * and no reset line: a program writes it and it is written. EXTEN_CTR2's
 * four low bits (33.2.2) are the amplifiers' HIGH-SPEED modes, OPAn_HSMD -
 * the CH32V303's datasheet gives the pair of rating tables 4-46-1 and
 * 4-46-2 (a unity-gain bandwidth of 19 MHz and 53 MHz, a slew rate of 8 and
 * 16 V/us, 195 and 770 uA) - on the CH32V30x_D8 and only on lots whose
 * penultimate sixth digit is not zero, which a program cannot read.
 *
 * AND ON A LOT WITHOUT THE REGISTER ITS ADDRESS IS NOT EMPTY: IT MIRRORS
 * EXTEN_CTR. Measured on the CH32V303VCT6 of the reference suite: a read
 * of 0x40023808 returns EXTEN_CTR's word, and a write there lands in
 * EXTEN_CTR - its USB bits in the low nibble the high-speed bits would
 * occupy, and the regulator trims above them. So no verb here writes the
 * address before `opa_high_speed_present()` has told the two apart: the
 * real register has nothing above bit 3 and reads unlike EXTEN_CTR, the
 * mirror reads EXTEN_CTR's own word, and the high-speed verbs answer
 * false having written nothing where it is the mirror.
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/device.hpp"
#include "ch32vx03/pin.hpp"

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

/// Whether this part's class has the upper half of OPA_CTLR and the
/// high-speed bits of EXTEN_CTR2: the CH32V30x_D8's (30.3.1's and 33.2.2's
/// notes).
inline constexpr bool opa_class_has_four = device::device_class == DeviceClass::v30x_d8;

/// Whether THIS DIE has EXTEN_CTR2 - a lot's register, and on a lot without
/// it the address mirrors EXTEN_CTR (the file header). The real register
/// keeps bits [31:4] clear and reads unlike EXTEN_CTR, whose regulator
/// trims stand above bit 3; the mirror reads EXTEN_CTR's own word. False
/// on every class but the CH32V30x_D8, without a read.
inline bool opa_high_speed_present() {
    if constexpr (!opa_class_has_four) {
        return false;
    } else {
        const uint32_t two = exten()->CTR2;
        return (two & ~0xFUL) == 0u && two != exten()->CTR;
    }
}

// =============================================================================
// The vocabulary
// =============================================================================

/// The six pads an amplifier can reach, named as the datasheets' pin
/// tables name them (OPAx_CH0P, OPAx_CH1N, OPAx_OUT1 and their kin).
/// One enum for all three roles, because a configuration that put an
/// output code in a positive input's place would otherwise be spelled
/// and only then refused.
enum class OpaPin : uint8_t {
    positive0,   ///< CH0P - PSEL = 0
    positive1,   ///< CH1P - PSEL = 1
    negative0,   ///< CH0N - NSEL = 0
    negative1,   ///< CH1N - NSEL = 1
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
 * The pad map, per DEVICE CLASS (the file header): the CH32V203's
 * datasheet 3.2 pin tables and the CH32V303's table 3-4. What a package
 * BONDS is the part's. An invalid pad for an amplifier the class has not
 * got.
 */
constexpr Pad opa_pad(uint8_t n, OpaPin which) {
    if (n == 1u) {
        switch (which) {
            case OpaPin::positive0: return Pad{'B', 15};
            case OpaPin::positive1: return Pad{'B', 0};
            case OpaPin::negative0: return Pad{'B', 11};
            case OpaPin::negative1: return Pad{'A', 6};
            case OpaPin::out0:      return Pad{'A', 3};
            case OpaPin::out1:      return opa_class_has_four ? Pad{'E', 15} : Pad{'B', 1};
        }
    } else if (n == 2u) {
        switch (which) {
            case OpaPin::positive0: return Pad{'B', 14};
            case OpaPin::positive1: return Pad{'A', 7};
            case OpaPin::negative0: return Pad{'B', 10};
            case OpaPin::negative1: return Pad{'A', 5};
            case OpaPin::out0:      return Pad{'A', 2};
            case OpaPin::out1:      return opa_class_has_four ? Pad{'E', 14} : Pad{'A', 4};
        }
    } else if (n == 3u && opa_class_has_four) {
        switch (which) {
            case OpaPin::positive0: return Pad{'B', 13};
            case OpaPin::positive1: return Pad{'C', 5};
            case OpaPin::negative0: return Pad{'B', 2};
            case OpaPin::negative1: return Pad{'C', 2};
            case OpaPin::out0:      return Pad{'A', 1};
            case OpaPin::out1:      return Pad{'E', 7};
        }
    } else if (n == 4u && opa_class_has_four) {
        switch (which) {
            case OpaPin::positive0: return Pad{'B', 12};
            case OpaPin::positive1: return Pad{'C', 4};
            case OpaPin::negative0: return Pad{'B', 1};
            case OpaPin::negative1: return Pad{'C', 3};
            case OpaPin::out0:      return Pad{'A', 0};
            case OpaPin::out1:      return Pad{'E', 8};
        }
    }
    return Pad{};
}

/// The ADC channel an output pad carries, or 0xFF where the pad is not a
/// converter input - the CH32V303's port-E outputs. ch32vx03/adc.hpp's own
/// map, repeated here as the few numbers this chapter needs rather than as
/// a dependency: the OPA does not otherwise know the converter.
constexpr uint8_t opa_output_channel(uint8_t n, OpaPin which) {
    const Pad p = opa_pad(n, which);
    if (!opa_is_output(which) || !p.valid()) {
        return 0xFF;
    }
    if (p.port == 'A' && p.pin <= 7u) {
        return p.pin;
    }
    if (p.port == 'B' && p.pin <= 1u) {
        return static_cast<uint8_t>(8u + p.pin);
    }
    return 0xFF;
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
                  "brio OpaIn: this part has no such amplifier (the CH32V203's table 2-1: two "
                  "on every part but the twenty-pin one, which counts one; the CH32V303's table "
                  "2-1-1: four)");
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
 * channel it carries - 0xFF for a pad that is not a converter input.
 * `claim()` puts the pad in analog mode, which is what the converter reads
 * it through.
 */
template <uint8_t n, OpaPin which>
struct OpaOut {
    static_assert(device::has_opa(n),
                  "brio OpaOut: this part has no such amplifier (the datasheets' tables 2-1 and "
                  "2-1-1)");
    static_assert(opa_is_output(which), "brio OpaOut: an output pad is out0 or out1");
    static constexpr uint8_t amplifier = n;
    static constexpr Pad pad = opa_pad(n, which);
    static constexpr uint8_t adc_channel = opa_output_channel(n, which);
    /// Whether the converter can read this pad at all.
    static constexpr bool reaches_adc = adc_channel != 0xFFu;
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

    static_assert(n >= 1u && n <= 4u,
                  "brio Opa: OPA_CTLR carries four amplifiers, OPA1..OPA4 (RM 30.3.1)");
    static_assert(n <= 2u || opa_class_has_four,
                  "brio Opa: OPA3 and OPA4 are the CH32V303's - every one of their bits names "
                  "the CH32V30x_D8 among other classes and no CH32V20x (RM 30.3.1)");
    static_assert(device::has_opa(n),
                  "brio Opa: this part has no such amplifier (the CH32V203's table 2-1: the "
                  "twenty-pin part bonds neither of OPA1's positive inputs and counts one)");

    static constexpr uint8_t instance = n;
    static constexpr uint32_t shift = opa_shift_for(n);
    /// Whether this amplifier has a high-speed bit (EXTEN_CTR2, 33.2.2).
    static constexpr bool has_high_speed = opa_class_has_four;

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
    /// The ADC channel the output currently selected reaches, 0xFF for
    /// none.
    static uint8_t adc_channel() { return opa_output_channel(n, configuration().output); }

    /**
     * OPAn_HSMD (EXTEN_CTR2, 33.2.2): the high-speed mode - the CH32V303
     * datasheet's table 4-46-2 against 4-46-1. The CH32V30x_D8's alone,
     * refused at compile time elsewhere; and on a DIE without the register
     * the verb writes NOTHING and answers false, because there the address
     * is EXTEN_CTR's mirror (the file header). True when the bit reads back
     * as asked.
     */
    static bool high_speed(bool on) {
        static_assert(has_high_speed,
                      "brio Opa: EXTEN_CTR2's high-speed bits are the CH32V30x_D8's (RM 33.2.2)");
        if (!opa_high_speed_present()) {
            return false;
        }
        const uint32_t bit = exten2_opa_hsmd(n);
        exten()->CTR2 = on ? (exten()->CTR2 | bit) : (exten()->CTR2 & ~bit);
        return high_speed() == on;
    }
    static bool high_speed() {
        static_assert(has_high_speed,
                      "brio Opa: EXTEN_CTR2's high-speed bits are the CH32V30x_D8's (RM 33.2.2)");
        return opa_high_speed_present() && (exten()->CTR2 & exten2_opa_hsmd(n)) != 0u;
    }

    /// Back to the reset state, this amplifier's bits alone (and its
    /// high-speed bit, where the die has one).
    static void release() {
        regs().CTLR = regs().CTLR & ~(opa_field_mask << shift);
        if constexpr (has_high_speed) {
            if (opa_high_speed_present()) {
                exten()->CTR2 = exten()->CTR2 & ~exten2_opa_hsmd(n);
            }
        }
    }

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
static_assert(opa_shift_for(3) == 8u && opa_shift_for(4) == 12u);
// The pads both series share.
static_assert(opa_pad(1, OpaPin::positive0) == Pad{'B', 15});
static_assert(opa_pad(1, OpaPin::positive1) == Pad{'B', 0});
static_assert(opa_pad(1, OpaPin::negative0) == Pad{'B', 11});
static_assert(opa_pad(1, OpaPin::negative1) == Pad{'A', 6});
static_assert(opa_pad(1, OpaPin::out0) == Pad{'A', 3});
static_assert(opa_pad(2, OpaPin::positive0) == Pad{'B', 14});
static_assert(opa_pad(2, OpaPin::positive1) == Pad{'A', 7});
static_assert(opa_pad(2, OpaPin::negative0) == Pad{'B', 10});
static_assert(opa_pad(2, OpaPin::negative1) == Pad{'A', 5});
static_assert(opa_pad(2, OpaPin::out0) == Pad{'A', 2});
// The second outputs, which are the class's.
static_assert(opa_pad(1, OpaPin::out1) == (opa_class_has_four ? Pad{'E', 15} : Pad{'B', 1}));
static_assert(opa_pad(2, OpaPin::out1) == (opa_class_has_four ? Pad{'E', 14} : Pad{'A', 4}));
static_assert(opa_pad(3, OpaPin::out0).valid() == opa_class_has_four);
static_assert(!opa_pad(5, OpaPin::out0).valid());
// Every OUT0 is a converter input; OUT1 is one on the CH32V203 alone.
static_assert(opa_output_channel(1, OpaPin::out0) == 3u);
static_assert(opa_output_channel(2, OpaPin::out0) == 2u);
static_assert(opa_output_channel(1, OpaPin::out1) == (opa_class_has_four ? 0xFFu : 9u));
static_assert(opa_output_channel(2, OpaPin::out1) == (opa_class_has_four ? 0xFFu : 4u));
static_assert(opa_output_channel(1, OpaPin::positive0) == 0xFFu);
static_assert(opa_config_valid(OpaConfig{}));
static_assert(!opa_config_valid(OpaConfig{.positive = OpaPin::out0}));
static_assert(!opa_config_valid(OpaConfig{.negative = OpaPin::positive0}));
static_assert(!opa_config_valid(OpaConfig{.output = OpaPin::negative1}));
static_assert(opa_config_bits(OpaConfig{}) == 0u);
static_assert(opa_config_bits(OpaConfig{.positive = OpaPin::positive1,
                                        .negative = OpaPin::negative1,
                                        .output = OpaPin::out1}) == 0xEu);
static_assert(exten2_opa_hsmd(1) == 1u && exten2_opa_hsmd(4) == 8u);

} // namespace brio
