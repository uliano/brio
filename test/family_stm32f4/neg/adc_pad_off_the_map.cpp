// PB5 is not one of the sixteen ADC1/ADC2 inputs, so no channel can be
// derived from the pad: the claim is refused rather than pointed at
// whatever channel a default would land on.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/adc.hpp"
void f() { (void)brio::AnalogIn<brio::Pin<'B', 5>>::channel; }
