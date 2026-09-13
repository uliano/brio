// WUT is sixteen bits; the seventeenth comes from WUCKSEL, not from a
// bigger reload.
// mcu: stm32f411xe stm32f429xx
#include "stm32f4/rtc.hpp"
void f() { (void)brio::Rtc::set_wakeup<brio::RtcWakeupClock::div16, 0x10000>(); }
