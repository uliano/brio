// PWM input mode watches one input with TWO channels, so a one-channel
// timer cannot carry it.
// mcu: stm32f411xe stm32f429xx stm32f446xx stm32f410cx
#include "stm32f4/tim.hpp"
void f() { (void)brio::TimPeriodMeter<brio::Tim<11>>::setup(); }
