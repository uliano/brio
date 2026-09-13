// TIM9's SMS codes 001..011 are Reserved (RM0383 14.4.2), so it has a
// slave controller and no quadrature interface.
// mcu: stm32f411xe stm32f429xx stm32f446xx
#include "stm32f4/tim.hpp"
void f() { (void)brio::TimEncoder<brio::Tim<9>>::setup(); }
