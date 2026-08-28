// mcu: samc21e18a samc21g18a samc21j18a
// CTRLC.WINMODE (43.8.3) implements seven modes; 0x7 is Reserved.

#include "samc/tsens.hpp"

using namespace brio;

constexpr TsensConfig bad_cfg{
    .calibration = TsensCalibration{.gain = 3000, .offset = -27315},
    .window = static_cast<TsensWindow>(7),
};

void use() { (void)Tsens::init<bad_cfg>(0); }
