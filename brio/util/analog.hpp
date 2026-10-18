/*
 * analog.hpp (util)
 *
 * The target-independent arithmetic of the analog block: counts <->
 * millivolts for any converter, given its full scale and the reference
 * in millivolts. Pure, constexpr, host-tested (test/test_analog).
 *
 * What is NOT here, on purpose: the reference levels (the enum Ref and
 * ref_mv() are each target's - one family's VREF offers 1.024/2.048/
 * 2.5/4.096 V, another a different set - so every target's vref header
 * defines its own brio::Ref under the same name, as with Clock or Pin:
 * two targets never meet in one binary) and the temperature formula
 * (calibration factors and their meaning are the silicon's, so the
 * conversion lives with the target's own converter).
 */

#pragma once

#include <stdint.h>

namespace brio {

/// Unsigned ADC counts -> millivolts, for a full-scale of `steps`
/// (4096 for 12 bits, 1024 for 10; multiply by the accumulation count
/// for accumulated results). Rounded to nearest. The product is formed
/// in 32 bits: exact for every 16-bit result (counts <= 65535, steps <=
/// 65536); a wider sum - the unscaled accumulation of 32 or more 12-bit
/// samples some converters deliver - goes to adc_sum_mv().
constexpr uint16_t adc_mv(uint32_t counts, uint32_t steps, uint16_t ref_mv_) {
    return static_cast<uint16_t>((counts * ref_mv_ + steps / 2) / steps);
}

/// adc_mv() for a sum of any width: the quotient and the remainder of
/// counts / steps scaled apart, the remainder's product in 64 bits (a
/// 22-bit remainder times a 16-bit reference does not fit 32). Rounded
/// to nearest. A cold-path conversion: a 64-bit multiply and divide.
constexpr uint16_t adc_sum_mv(uint32_t counts, uint32_t steps, uint16_t ref_mv_) {
    const uint32_t q = counts / steps;
    const uint64_t r = counts % steps;
    return static_cast<uint16_t>(q * ref_mv_ + (r * ref_mv_ + steps / 2) / steps);
}

/// adc_mv_signed() for a signed sum of any width (a differential
/// accumulation); `half_steps` = 2048 x the samples summed.
constexpr int16_t adc_sum_mv_signed(int32_t counts, uint32_t half_steps, uint16_t ref_mv_) {
    const uint32_t mag = counts >= 0 ? static_cast<uint32_t>(counts)
                                     : static_cast<uint32_t>(-static_cast<int64_t>(counts));
    const uint16_t mv = adc_sum_mv(mag, half_steps, ref_mv_);
    return counts >= 0 ? static_cast<int16_t>(mv) : static_cast<int16_t>(-static_cast<int32_t>(mv));
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
