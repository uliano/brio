// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 30.6.9: "CALM[1:0] are stuck at 00 when CALW8 = 1". A value the
// hardware would quietly round is a value the caller did not ask for.
#include "stm32g0/rtc.hpp"
static_assert(brio::rtc_calibration_valid(
    {.minus = 3, .window = brio::RtcCalibrationWindow::seconds8}));
