// This family numbers its converters 1..3 and no part of it has a fourth.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/adc.hpp"
void f() { (void)brio::Adc<4>::regs(); }
