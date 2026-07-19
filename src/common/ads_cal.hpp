#pragma once
#include "ads131m02.hpp"

// Board-specific ADS131M02 calibration (REF35102 = 1.024 V applied reference).
// Captured with the `adccal` app: offsets c0_* from the internal input short, codes
// cR_* read with 1.024 V on AINxP. Re-run adccal and update the four numbers below if
// the reference, wiring, or chip changes.
//
//   cal_from_two_point(c0_CH0, cR_CH0, c0_CH1, cR_CH1)
//
// Apps load it once after reset:  Adc::load_calibration(ads131::kBoardCal);

namespace ads131 {

constexpr Calibration kBoardCal = cal_from_two_point(662, 7244275, 800, 7295340);

}  // namespace ads131
