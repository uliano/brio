/*
 * dac.hpp
 *
 * Dac<n>: the AVR DA/DB 10-bit DAC (DS40002247B ch. 34) as a brio
 * actuator - rank of Pin: synchronous, set-and-forget, no events. One
 * instance on DA/DB (DAC0); the template parameter is there for the
 * symmetry every other driver has and for targets with several.
 *
 *   using Out = brio::Dac<0>;
 *   Out::init({.reference = brio::Ref::v2048});   // buffered out on PD6
 *   Out::set(512);                                // ~1.024 V
 *
 * What the silicon does: DATA[9:0] left-adjusted (DATA[9:2] in DATAH,
 * [1:0] in DATAL bits 7:6, output updated when DATAH is written - the
 * 16-bit write below does it in that order); range GND .. VDACREF
 * (VREF.DAC0REF); the buffered output on the DAC pin (PD6, `OUTEN`;
 * the pin's digital input must be disabled in PORT) sources 1 mA, sinks
 * ~1 uA (add a resistor to ground if it must sink), settles in ~7-10 us
 * full scale; the UNBUFFERED output feeds the ADC (MUXPOS DAC0), the
 * ACs and the OPAMPs internally with OUTEN off, leaving PD6 free.
 * RUNSTDBY keeps it running in standby sleep.
 *
 * Bench facts (analog0, A5 silicon, bare pin): the buffered output RISES
 * to a new code within ~10-20 us but FALLS at the sink limit - about
 * 1 uA into the pin capacitance, i.e. ~20 kV/s: from 2 V to 0 V takes
 * ~100 us or more, and the last tens of mV much longer. A falling step
 * that must be fast needs the datasheet's resistor to ground (10 kOhm
 * sinks 200 uA at 2 V: 100 x faster). Settling after a rising step
 * shows a ~50 us ring before the last few LSB settle. The buffered
 * output sits ~13 mV (26 ADC counts at 2.048 V) above the unbuffered
 * internal path: the buffer's offset (spec +-10 mV).
 *
 * Errata DS80000915F 2.6.1 (silicon A4/A5): the output buffer's offset
 * drifts over the device's lifetime if the part is powered with OUTEN
 * off. Rule of this driver: `output_pin` defaults to true and, once
 * enabled, the buffer stays on for the life of the program (disable()
 * turns the converter off, not the buffer). An app that truly wants
 * OUTEN off (internal use only) says so and owns the drift, or
 * calibrates the offset against the ADC.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrdx/pin.hpp"
#include "avrdx/vref.hpp"
#include "util/analog.hpp"

namespace brio {

struct DacConfig {
    Ref reference = Ref::vdd;
    bool output_pin = true;      ///< buffered output on the DAC pin (PD6)
    bool run_standby = false;
    bool reference_always_on = false;
};

template <uint8_t dac_num>
class Dac {
    static_assert(dac_num == 0, "AVR DA/DB have one DAC: DAC0");

public:
    Dac() = delete;

    static constexpr uint32_t steps = 1024;   ///< 10 bits
    using OutPin = Pin<'D', 6>;               ///< DAC0 OUT on this family

    /// Select the reference, configure the output, enable. The initial
    /// output is code 0. Call after clock init; the reference start-up
    /// (10-200 us) and the buffer settling follow the first set().
    static void init(DacConfig cfg) {
        Vref::dac0(cfg.reference, cfg.reference_always_on);
        ref_ = cfg.reference;
        if (cfg.output_pin) {
            OutPin::disable_digital_input();  // 34.3.1: input disabled on the DAC pin
        }
        regs().DATA = 0;
        regs().CTRLA = static_cast<uint8_t>(
            DAC_ENABLE_bm |
            (cfg.output_pin ? DAC_OUTEN_bm : 0) |
            (cfg.run_standby ? DAC_RUNSTDBY_bm : 0));
    }

    /// Output code 0..1023 (clamped). One 16-bit write, DATAL then
    /// DATAH: the output updates on the high byte.
    static void set(uint16_t code) {
        if (code > steps - 1) {
            code = steps - 1;
        }
        regs().DATA = static_cast<uint16_t>(code << 6);
    }

    /// Output a voltage, given the reference's millivolts (the app knows
    /// them: ref_mv(Ref::v2048), or the measured VDD).
    static void set_mv(uint16_t mv, uint16_t reference_mv) {
        set(dac_code(mv, steps, reference_mv));
    }

    /// The reference chosen at init (for whoever converts codes to volts).
    static Ref reference() { return ref_; }

    /// Converter off. The output buffer bit is left as configured (see
    /// the errata note); re-enable with enable().
    static void disable() { regs().CTRLA &= static_cast<uint8_t>(~DAC_ENABLE_bm); }
    static void enable() { regs().CTRLA |= DAC_ENABLE_bm; }

private:
    static constexpr DAC_t& regs() { return DAC0; }
    static inline Ref ref_ = Ref::vdd;
};

} // namespace brio
