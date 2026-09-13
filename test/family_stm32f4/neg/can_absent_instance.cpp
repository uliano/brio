// The F401, F410 and F411 carry no bxCAN, and their device header says so
// by declaring no CAN1_BASE.
// mcu: stm32f411xe stm32f401xe stm32f410cx
#include "stm32f4/can.hpp"
void f() { (void)brio::Can<1>::in_init(); }
