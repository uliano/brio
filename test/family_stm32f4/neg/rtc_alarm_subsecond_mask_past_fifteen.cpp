// MASKSS is four bits: an alarm asking for a sixteenth is refused.
// mcu: stm32f411xe stm32f429xx
#include "stm32f4/rtc.hpp"
void f() { (void)brio::Rtc::set_alarm<brio::RtcAlarm{.subsecond_mask = 16}>(brio::RtcAlarmId::a); }
