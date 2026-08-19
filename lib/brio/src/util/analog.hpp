/*
 * analog.hpp (util)
 *
 * The target-independent arithmetic of the analog block: reference
 * levels, counts <-> millivolts, temperature from calibration factors.
 * Pure, constexpr, host-tested (test/test_analog). No register knows
 * this file exists; the drivers of a target take a Ref and hand its
 * millivolts to whoever converts.
 */

#pragma once

#include <stdint.h>

namespace brio {

/// A voltage reference selection. The internal levels are the ones the
/// AVR DA/DB VREF offers; `vdd` and `vrefa` are "whatever it is" - the
/// app supplies the millivolts when it knows them.
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

/// Unsigned ADC counts -> millivolts, for a full-scale of `steps`
/// (4096 for 12 bits, 1024 for 10; multiply by the accumulation count
/// for accumulated results). Rounded to nearest.
constexpr uint16_t adc_mv(uint32_t counts, uint32_t steps, uint16_t ref_mv_) {
    return static_cast<uint16_t>((counts * ref_mv_ + steps / 2) / steps);
}

/// Signed (differential) ADC counts -> millivolts; `half_steps` = 2048
/// for 12 bits, 512 for 10.
constexpr int16_t adc_mv_signed(int32_t counts, uint32_t half_steps, uint16_t ref_mv_) {
    const int32_t num = counts * static_cast<int32_t>(ref_mv_);
    const int32_t den = static_cast<int32_t>(half_steps);
    return static_cast<int16_t>(num >= 0 ? (num + den / 2) / den : (num - den / 2) / den);
}

/// Millivolts -> DAC code for a `steps`-step converter (1024 for the
/// 10-bit DAC), saturating at full scale.
constexpr uint16_t dac_code(uint16_t mv, uint32_t steps, uint16_t ref_mv_) {
    if (ref_mv_ == 0) return 0;
    const uint32_t code = (static_cast<uint32_t>(mv) * steps + ref_mv_ / 2) / ref_mv_;
    return static_cast<uint16_t>(code >= steps ? steps - 1 : code);
}

/// DAC code -> millivolts (the voltage the converter aims at).
constexpr uint16_t dac_mv(uint16_t code, uint32_t steps, uint16_t ref_mv_) {
    return static_cast<uint16_t>((static_cast<uint32_t>(code) * ref_mv_ + steps / 2) / steps);
}

/// Die temperature in kelvin from a 12-bit single-ended reading of the
/// sensor with the 2.048 V reference and the two signature-row factors
/// (DS40002247B 33.3.3.8): T = (offset - result) * slope / 4096.
constexpr uint16_t temp_kelvin(uint16_t adc_result, uint16_t sigrow_slope,
                               uint16_t sigrow_offset) {
    uint32_t t = static_cast<uint32_t>(sigrow_offset) - adc_result;
    t *= sigrow_slope;
    t += 4096u / 2;
    t /= 4096u;
    return static_cast<uint16_t>(t);
}

} // namespace brio
