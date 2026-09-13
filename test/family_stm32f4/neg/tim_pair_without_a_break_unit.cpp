// A complementary pair with a dead time is the advanced-control timers'
// alone: TIM5 has four channels and no BDTR at all.
// mcu: stm32f411xe stm32f429xx stm32f446xx
#include "stm32f4/tim.hpp"
void f() { (void)brio::TimPairPwm<brio::Tim<5>, 0>::setup(); }
