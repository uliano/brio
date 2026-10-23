/*
 * vref.hpp
 *
 * What full scale means for the analog blocks of an AVR DA, DB or EA:
 * the reference each converter and the comparators measure against. A
 * program names its reference where it configures the ADC, the DAC or
 * a comparator (avrxt/adc.hpp, avrxt/dac.hpp, avrxt/ac.hpp), whose
 * init() writes the selection through the setters below, and converts
 * counts to millivolts with ref_mv() and util/analog.hpp. Called
 * directly, to keep a reference powered or to read its level:
 *
 *     brio::Vref::ac(brio::Ref::v1024, true);                  // the ACs' reference, always on
 *     constexpr uint16_t fs_mv = brio::ref_mv(brio::Ref::v2048);  // 2048
 *
 * This is a vocabulary plus three setters, not a device: the enum Ref
 * names this silicon's levels (another target's vref header defines
 * its own brio::Ref - same name, its levels), ref_mv() gives their
 * millivolts to whoever converts (util/analog.hpp holds the
 * target-independent arithmetic); the setters are called by the
 * consumers' init() - an app names its reference where it configures
 * the ADC or the DAC and never writes VREF itself.
 *
 * Which context calls what. ref_mv() and refsel() are constant
 * arithmetic, legal anywhere and folded at compile time. Each setter is
 * one store into its VREF register: it waits for nothing, masks
 * nothing, and is a setup verb of the consumer that owns that register
 * (its init(), from main() or from the active object that owns the
 * converter). Nothing here checks that VDD has the headroom a level
 * needs, nor that the reference a program converts with is the one it
 * selected.
 *
 * The voltage reference selector (VREF) of the AVR DA/DB (DS40002247B
 * ch. 21) and the AVR EA (DS40002443A ch. 20): one REFSEL per consumer -
 * ADC0 (DA/DB only), DAC0, the analog comparators - each picking 1.024 /
 * 2.048 / 2.500 / 4.096 V, VDD, or the external VREFA pin (PD7, 1.024 V
 * .. VDD). On the DA/DB the internal levels are +-4 % untrimmed, 2.048
 * needs VDD >= 2.5 V, 2.5 needs 2.95 V, 4.096 needs 4.55 V, and a source
 * starts in 10 us with a high-frequency main clock (200 us with a
 * 32.768 kHz one, Table 39-17; 2 us to change level), ~40-175 uA kept
 * on. On the EA the levels are +-2 % (1.024, 2.048) and +-3 % (2.5,
 * 4.096) to 85 C and need VDD >= 1.8 / 2.55 / 3.0 / 4.6 V (table 35-17),
 * a source starts in 130 us typ., 180 max (tVREF_ST); its ADC chooses
 * its reference in its own CTRLC, with its own encoding (adc.hpp's
 * adc_refsel(): never refsel() below). A source turns on when its
 * consumer needs it; ALWAYSON keeps it on.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrxt/device_tables.hpp"
#include "util/analog.hpp"

namespace brio {

/// A voltage reference selection: the internal levels the VREF offers;
/// `vdd` and `vrefa` are "whatever it is" - the app supplies the
/// millivolts when it knows them.
enum class Ref : uint8_t { v1024, v2048, v2500, v4096, vdd, vrefa };

/// Millivolts of a reference. For vdd/vrefa the app's known value is
/// returned as given (0 = unknown, conversions then yield 0).
constexpr uint16_t ref_mv(Ref r, uint16_t known_mv = 0) {
    switch (r) {
    case Ref::v1024: return 1024;
    case Ref::v2048: return 2048;
    case Ref::v2500: return 2500;
    case Ref::v4096: return 4096;
    case Ref::vdd:
    case Ref::vrefa: return known_mv;
    }
    return 0;
}

struct Vref {
    Vref() = delete;

#if defined(VREF_ADC0REF)
    /// The ADC's reference, where the VREF block chooses it (device
    /// tables' has_vref_adc0ref).
    static void adc0(Ref r, bool always_on = false) { VREF.ADC0REF = bits(r, always_on); }
#endif
    /// The DAC's reference (VREF.DAC0REF), one store; `always_on` sets
    /// ALWAYSON, the source powered whether the DAC asks for it or not.
    static void dac0(Ref r, bool always_on = false) { VREF.DAC0REF = bits(r, always_on); }
    /// The analog comparators' shared reference (VREF.ACREF, the one
    /// their DACREF divides), one store; `always_on` as above.
    static void ac(Ref r, bool always_on = false) { VREF.ACREF = bits(r, always_on); }

    static constexpr uint8_t refsel(Ref r) {
        switch (r) {
        case Ref::v1024: return VREF_REFSEL_1V024_gc;
        case Ref::v2048: return VREF_REFSEL_2V048_gc;
        case Ref::v2500: return VREF_REFSEL_2V500_gc;
        case Ref::v4096: return VREF_REFSEL_4V096_gc;
        case Ref::vdd: return VREF_REFSEL_VDD_gc;
        case Ref::vrefa: return VREF_REFSEL_VREFA_gc;
        }
        return VREF_REFSEL_1V024_gc;
    }

private:
    static constexpr uint8_t bits(Ref r, bool always_on) {
        return static_cast<uint8_t>(refsel(r) | (always_on ? VREF_ALWAYSON_bm : 0));
    }
};

} // namespace brio
