// Counting a trigger is external clock mode 1, and TIM11 has no slave
// mode control register.
// mcu: stm32f411xe stm32f429xx stm32f446xx stm32f410cx
#include "stm32f4/tim.hpp"
void f() { (void)brio::TimEventCounter<brio::Tim<11>>::setup(brio::TimTrigger::itr0); }
