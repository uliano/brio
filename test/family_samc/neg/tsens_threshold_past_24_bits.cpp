// mcu: samc21e18a samc21g18a samc21j18a
// WINLT and WINUT are 24-bit two's-complement fields (43.8.11/43.8.12),
// the same width as the VALUE they are compared against. A threshold
// past that field would be silently truncated into a different one.

#include "samc/tsens.hpp"

using namespace brio;

constexpr TsensConfig bad_cfg{
    .calibration = TsensCalibration{.gain = 3000, .offset = -27315},
    .window = TsensWindow::above,
    .window_lower = tsens_value_max + 1,
};

void use() { (void)Tsens::init<bad_cfg>(0); }
