// PREDIV_A is seven bits.
// mcu: stm32f411xe stm32f429xx
#include "stm32f4/rtc.hpp"
void f() { (void)brio::Rtc::set_prescalers<brio::RtcPrescalers{.async = 128}>(); }
