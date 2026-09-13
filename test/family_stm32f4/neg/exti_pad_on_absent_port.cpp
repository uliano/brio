// SYSCFG_EXTICR has a code for every port the part bonds and none for a
// port it has not got, so a line reached through such a pad is refused.
// mcu: stm32f411xe stm32f401xe stm32f410cx
#include "stm32f4/exti.hpp"
void f() { (void)brio::ExtInt<brio::Pin<'F', 3>>::claim(); }
