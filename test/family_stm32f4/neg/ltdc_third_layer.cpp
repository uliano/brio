// Two layers, 1 at the bottom and 2 on top (16.4.2). There is no third,
// and a program that asks for one is asking for a register block that
// does not exist.
// mcu: stm32f429xx stm32f439xx stm32f469xx
#include "stm32f4/ltdc.hpp"
void f() { (void)&brio::LtdcLayer<3>::regs(); }
