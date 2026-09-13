// USART3 is the F405 class's and up; the F411 has none.
// mcu: stm32f411xe stm32f401xe
#include "stm32f4/usart.hpp"
void f() { brio::Usart<3>::enable(true); }
