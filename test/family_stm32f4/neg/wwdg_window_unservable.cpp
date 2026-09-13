// A WWDG window below 0x40 can never be served - a refresh is legal only
// while the counter is at or below W and above 0x3F (RM0090 22.3) - so
// it is refused instead of arming a watchdog no loop could feed.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/reset.hpp"
void f() { (void)brio::Wwdg::configure<brio::WwdgConfig{brio::WwdgPrescaler::div1, 0x3F}>(); }
