/*
 * analog.hpp (util)
 *
 * The target-independent arithmetic of the analog block: counts <->
 * millivolts for any converter, given its full scale and the reference
 * in millivolts. Pure, constexpr, host-tested (test/test_analog).
 *
 * What is NOT here, on purpose: the reference levels (the enum Ref and
 * ref_mv() are each target's - the AVR DA/DB VREF offers 1.024/2.048/
 * 2.5/4.096 V, other silicon offers other sets - so every target's
 * vref header defines its own brio::Ref under the same name, as with
 * Clock or Pin: two targets never meet in one binary) and the
 * temperature formula (calibration factors and their meaning are the
 * silicon's: Adc<n>::temp_kelvin() on AVR DA/DB).
 */

#pragma once

#include <stdint.h>

namespace brio {

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

} // namespace brio
