// mcu: samc21e18a samc21g18a samc21j18a
// EVCTRL.STARTINV (43.8.4) inverts the START event input; with
// EVCTRL.STARTEI clear there is no input to invert.

#include "samc/tsens.hpp"

using namespace brio;

constexpr TsensConfig bad_cfg{
    .calibration = TsensCalibration{.gain = 3000, .offset = -27315},
    .events = TsensEventControl{.invert_start = true},
};

void use() { (void)Tsens::init<bad_cfg>(0); }
