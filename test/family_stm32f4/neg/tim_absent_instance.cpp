// TIM8 is the second advanced-control timer, which the F401, F410 and
// F411 have not got - and there is no TIM15 anywhere in this family.
// mcu: stm32f411xe stm32f401xc stm32f410cx
#include "stm32f4/tim.hpp"
void f() { brio::Tim<8>::init(); }
