// mcu: stm32g030xx stm32g050xx stm32g070xx stm32g0b0xx
// TIM2 - the family's one 32-bit counter - is the x1 line's: no x0
// header declares TIM2_BASE, so a Tim<2> there is the "no such timer"
// refusal, and tim_irq(2) has no TIM2_IRQn to name.
#include "stm32g0/tim.hpp"
void f() { brio::Tim<2>::init(); }
