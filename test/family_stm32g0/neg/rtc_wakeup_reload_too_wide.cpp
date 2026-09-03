// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// WUT[15:0] is a sixteen-bit field (30.6.6); a caller asking for more
// has asked for a different timer, not a slower one.
#include "stm32g0/rtc.hpp"
static_assert(brio::rtc_wakeup_valid(brio::RtcWakeupClock::div16, 0x10000));
