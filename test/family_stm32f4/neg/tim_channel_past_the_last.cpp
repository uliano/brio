// TIM11 has ONE capture/compare channel; asking for its second is a
// register that does not exist.
// mcu: stm32f411xe stm32f429xx stm32f446xx stm32f410cx
#include "stm32f4/tim.hpp"
void f() { (void)brio::TimPwm<brio::Tim<11>, 1>::setup(); }
