// The last EXTI line of a part is the last one: a line number past it is
// not a line to be masked at run time, it is a program that means nothing.
// mcu: stm32f429xx stm32f446xx stm32f411xe stm32f410tx
#include "stm32f4/exti.hpp"
void f() { (void)brio::ExtiLine<24>::arm(true); }
