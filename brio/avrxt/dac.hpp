/*
 * dac.hpp
 *
 * Dac<n>: a voltage out of the chip - a bias, a test level, a slow
 * waveform, or a level fed inside the chip to the ADC or a comparator.
 * The 10-bit DAC of the AVR DA/DB (DS40002247B ch. 34) and of the AVR
 * EA (DS40002443A ch. 32) - one IP - as a brio actuator of the rank of
 * Pin: synchronous, set-and-forget, no events, no interrupt. One
 * instance (DAC0); the template parameter is there for the symmetry
 * every other driver has and for targets with several. init() selects
 * the reference and configures the output, set() writes a code and the
 * output follows:
 *
 *   using Out = brio::Dac<0>;
 *   Out::init({.reference = brio::Ref::v2048});   // buffered out on PD6
 *   Out::set(512);                                // ~1.024 V
 *
 * Which context calls what. init(), disable() and enable() are setup
 * verbs of the one owner (main(), or the active object that owns the
 * converter): register writes that return at once - init() writes
 * VREF.DAC0REF, PD6's pin control, DATA and CTRLA, disable() and
 * enable() rewrite CTRLA (a read-modify-write on the DA/DB) - and none
 * takes the clock, waits or masks interrupts. set() is one 16-bit store
 * and code() one 16-bit load, from the owner's context. Nothing waits
 * for the reference's start-up or the buffer's settling: they follow in
 * the analog domain, and nothing checks that VDD has the headroom the
 * reference needs (docs/avrxt/vref.md).
 *
 * What the silicon does: DATA[9:0] left-adjusted (DATA[9:2] in DATAH,
 * [1:0] in DATAL bits 7:6, output updated when DATAH is written - the
 * 16-bit write below does it in that order); range GND .. VDACREF
 * (VREF.DAC0REF); the buffered output on the DAC pin (PD6, `OUTEN`;
 * the pin's digital input must be disabled in PORT) sources 1 mA, sinks
 * ~1 uA (add a resistor to ground if it must sink), settles in ~7-10 us
 * full scale - the DB data sheet's figures (39.19); the EA's table 35-22
 * gives no drive figure and its buffer was measured sinking milliamperes
 * (docs/avrxt/dac.md); the unbuffered output feeds the ADC (MUXPOS DAC0) and
 * the ACs internally with OUTEN off, leaving PD6 free - the OPAMPs
 * instead tap the buffered output (34.3.2.3): feeding them needs
 * OUTEN on, and PD6 is then taken. On the EA the unbuffered output
 * feeds the ADC alone (32.3.2.3): its comparators reach the DAC only
 * through PD6 (AINP3), so OUTEN on. The EA adds OUTRANGE (32.5.1), the
 * output buffer optimized for low codes, high codes, or toggled between
 * the two by DATA's MSb - `range`.
 * RUNSTDBY keeps it running in standby sleep.
 *
 * Bench facts (the AVR DB's A5 silicon, bare pin; the AVR DA's and the
 * AVR EA's are in docs/avrxt/dac.md): the buffered output rises to a
 * new code within ~10-20 us but falls at the sink limit - about 1 uA
 * into the pin capacitance, i.e. ~20 kV/s: from 2 V to 0 V takes
 * ~100 us or more, and the last tens of mV much longer. A falling step
 * that must be fast needs the datasheet's resistor to ground (10 kOhm
 * sinks 200 uA at 2 V: 100 x faster). Settling after a rising step
 * shows a ~50 us ring before the last few LSB settle. The buffered
 * output sits ~13 mV (26 ADC counts at 2.048 V) above the unbuffered
 * internal path: the buffer's offset (spec +-10 mV).
 *
 * Errata DS80000915F 2.6.1 (silicon A4/A5) and DS80000882C 2.5.1: the
 * output buffer's offset drifts over the device's lifetime if the part
 * is powered with OUTEN off. Rule of this driver on the AVR DA/DB
 * (device tables' erratum_dac_outen_drift): `output_pin` defaults to
 * true and, once enabled, the buffer stays on for the life of the
 * program (disable() turns the converter off, not the buffer). An app
 * that truly wants OUTEN off (internal use only) says so and owns the
 * drift, or calibrates the offset against the ADC. DS80001048D has no
 * DAC item: on the AVR EA disable() turns the buffer off too and the
 * pin is the port's again; enable() restores what init() configured.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrxt/device_tables.hpp"
#include "avrxt/pin.hpp"
#include "avrxt/vref.hpp"
#include "util/analog.hpp"

namespace brio {

#if defined(DAC_OUTRANGE_gm)
/// The output buffer's range (CTRLA.OUTRANGE, DS40002443A 32.5.1): `full`
/// toggles between the low and the high range on DATA's MSb; `low` and
/// `high` hold one. The codes are the device header's (it says AUTO 0,
/// LOW 2, HIGH 3; the data sheet's 32.5.1 says FULL 0, LOW 1, HIGH 2 -
/// docs/avrxt/dac.md has the bench's reading).
enum class DacRange : uint8_t {
    full = DAC_OUTRANGE_AUTO_gc,
    low = DAC_OUTRANGE_LOW_gc,
    high = DAC_OUTRANGE_HIGH_gc,
};
#endif

struct DacConfig {
    Ref reference = Ref::vdd;
    bool output_pin = true;      ///< buffered output on the DAC pin (PD6)
    bool run_standby = false;
    bool reference_always_on = false;
#if defined(DAC_OUTRANGE_gm)
    DacRange range = DacRange::full;
#endif
};

template <uint8_t dac_num>
class Dac {
    static_assert(dac_num == 0, "AVR DA/DB have one DAC: DAC0");

public:
    Dac() = delete;

    static constexpr uint32_t steps = 1024;   ///< 10 bits
    using OutPin = Pin<'D', 6>;               ///< DAC0 OUT on this family

    /// Select the reference, configure the output, enable. The initial
    /// output is code 0. A setup verb that takes no clock and reads none;
    /// it returns at once, and the reference start-up (10-200 us on the
    /// DA/DB, 130 us typ. and 180 max on the EA) and the buffer settling
    /// follow in the analog domain. With `output_pin`, PD6's digital
    /// input buffer is turned off and the pad is the DAC's.
    static void init(DacConfig cfg) {
        Vref::dac0(cfg.reference, cfg.reference_always_on);
        ref_ = cfg.reference;
        if (cfg.output_pin) {
            OutPin::input_enable(false);      // 34.3.1: input disabled on the DAC pin
        }
        regs().DATA = 0;
        regs().CTRLA = static_cast<uint8_t>(
            DAC_ENABLE_bm |
            (cfg.output_pin ? DAC_OUTEN_bm : 0) |
#if defined(DAC_OUTRANGE_gm)
            static_cast<uint8_t>(cfg.range) |
#endif
            (cfg.run_standby ? DAC_RUNSTDBY_bm : 0));
        if constexpr (!erratum_dac_outen_drift) {
            ctrla_ = regs().CTRLA;
        }
    }

    /// The code currently in DATA (what set() last wrote).
    static uint16_t code() { return static_cast<uint16_t>(regs().DATA >> 6); }

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

    /// Converter off. On the AVR DA/DB the output buffer bit is left as
    /// configured (the errata note); on the AVR EA the buffer goes off with
    /// it and the pin is released. enable() restores what init() set.
    static void disable() {
        if constexpr (erratum_dac_outen_drift) {
            regs().CTRLA &= static_cast<uint8_t>(~DAC_ENABLE_bm);
        } else {
            regs().CTRLA = static_cast<uint8_t>(ctrla_ & ~(DAC_ENABLE_bm | DAC_OUTEN_bm));
        }
    }
    static void enable() {
        if constexpr (erratum_dac_outen_drift) {
            regs().CTRLA |= DAC_ENABLE_bm;
        } else {
            regs().CTRLA = ctrla_;
        }
    }

    /// The register block, for a suite's silicon questions.
    static constexpr DAC_t& regs() { return DAC0; }

private:
    static inline Ref ref_ = Ref::vdd;
    /// CTRLA as init() wrote it (the AVR EA's enable() restores it).
    static inline uint8_t ctrla_ = 0;
};

} // namespace brio
