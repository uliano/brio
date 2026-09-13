// A PwmChannel's max is its period, and a period of zero has no duty to
// set - and would block the counter besides (RM0090 17.4.12).
// mcu: stm32f411xe stm32f429xx stm32f446xx stm32f410cx
#include "stm32f4/tim.hpp"
void f() { (void)brio::TimPwm<brio::Tim<5>, 0, 0>::setup(); }
