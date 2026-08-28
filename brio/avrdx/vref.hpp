/*
 * vref.hpp
 *
 * The AVR DA/DB voltage reference selector (VREF, DS40002247B ch. 21):
 * one REFSEL per consumer - ADC0, DAC0, the analog comparators - each
 * picking 1.024 / 2.048 / 2.500 / 4.096 V (internal, +-4 % untrimmed;
 * 2.048 needs VDD >= 2.5 V, 2.5 needs 2.95 V, 4.096 needs 4.55 V), VDD,
 * or the external VREFA pin (PD7, 1.024 V .. VDD). A source turns on
 * when its consumer needs it; ALWAYSON keeps it on (start-up 10 us
 * with a high-frequency main clock - OSCHF or EXTCLK/XOSCHF - and
 * 200 us with a 32.768 kHz one, Table 39-17; 2 us to change level)
 * for ~40-175 uA.
 *
 * This is a vocabulary plus three setters, not a device: the enum Ref
 * names this silicon's levels (another target's vref header defines
 * its own brio::Ref - same name, its levels), ref_mv() gives their
 * millivolts to whoever converts (util/analog.hpp holds the
 * target-independent arithmetic); the setters are called by the
 * consumers' init() - an app names its reference where it configures
 * the ADC or the DAC and never writes VREF itself.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

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

    static void adc0(Ref r, bool always_on = false) { VREF.ADC0REF = bits(r, always_on); }
    static void dac0(Ref r, bool always_on = false) { VREF.DAC0REF = bits(r, always_on); }
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
