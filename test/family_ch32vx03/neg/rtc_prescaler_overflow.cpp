// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// AN RTC PRESCALER PAST ITS FIELD. RTC_PSCR is twenty bits and the
// division it makes is the value plus one (RM 6.3.4), so 2^20 is the
// longest tick this block can make; a reload above that is refused
// where the configuration is a constant.
#include "ch32vx03/rtc.hpp"

void f() {
    (void)brio::Rtc::configure<brio::RtcConfig{.prescaler = brio::rtc_prescaler_max + 1u}>();
}
