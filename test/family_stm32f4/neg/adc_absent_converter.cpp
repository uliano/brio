// ADC2 and ADC3 are the F405 class's, the F42x/F43x's, the F446's and the
// F469/F479's; every other part has ADC1 alone and its header says so.
// mcu: stm32f411xe stm32f401xe stm32f410cx stm32f412zx stm32f413xx
#include "stm32f4/adc.hpp"
void f() { (void)brio::Adc<2>::regs(); }
