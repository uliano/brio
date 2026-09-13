// Twenty backup registers, so there is no twenty-first.
// mcu: stm32f411xe stm32f429xx
#include "stm32f4/rtc.hpp"
void f() { (void)brio::Rtc::backup<20>(); }
