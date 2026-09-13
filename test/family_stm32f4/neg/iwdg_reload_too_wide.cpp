// IWDG_RLR.RL is twelve bits (RM0090 21.4.3): a reload that does not fit
// is refused at compile time rather than silently truncated to a
// time-out nobody asked for.
// mcu: stm32f429xx stm32f411xe
#include "stm32f4/reset.hpp"
void f() { (void)brio::Iwdg::configure<brio::IwdgConfig{brio::IwdgPrescaler::div4, 0x1000}>(); }
