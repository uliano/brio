// mcu: samc21e18a samc21g18a samc21j18a
// WINMODE INSIDE is "WINLT < VALUE < WINUT" (43.8.3) in both documents,
// so a lower threshold above the upper one is a window that can never
// open. (The same refusal does NOT apply to OUTSIDE, which the datasheet
// and the device header describe with the thresholds in opposite
// orders - see tsens.hpp.)

#include "samc/tsens.hpp"

using namespace brio;

constexpr TsensConfig bad_cfg{
    .calibration = TsensCalibration{.gain = 3000, .offset = -27315},
    .window = TsensWindow::inside,
    .window_lower = 6000,
    .window_upper = 2000,
};

void use() { (void)Tsens::init<bad_cfg>(0); }
