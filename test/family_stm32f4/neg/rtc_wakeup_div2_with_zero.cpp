// RM0383 17.6.6: "setting WUT[15:0] to 0x0000 with WUCKSEL[2:0] = 011
// (RTCCLK/2) is forbidden".
// mcu: stm32f411xe stm32f429xx
#include "stm32f4/rtc.hpp"
void f() { (void)brio::Rtc::set_wakeup<brio::RtcWakeupClock::div2, 0>(); }
