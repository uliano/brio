// RM0383 17.6.16: CALM[0] is stuck at zero in the 16-second window, so an
// odd CALM there is a value the hardware would silently round.
// mcu: stm32f411xe stm32f429xx
#include "stm32f4/rtc.hpp"
void f() {
    (void)brio::Rtc::calibrate<brio::RtcCalibration{
        .minus = 1, .window = brio::RtcCalibrationWindow::seconds16}>();
}
