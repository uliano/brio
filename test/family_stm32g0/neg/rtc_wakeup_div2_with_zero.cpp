// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 30.6.6, in those words: "Setting WUT[15:0] to 0x0000 with
// WUCKSEL[2:0] = 011 (RTCCLK/2) is forbidden."
#include "stm32g0/rtc.hpp"
static_assert(brio::rtc_wakeup_valid(brio::RtcWakeupClock::div2, 0));
