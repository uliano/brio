// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb
// TIM8'S TRGO AS A TRIGGER ON A PART WITHOUT A TIM8. Code 110 is EXTI line
// 11 until AFIO's ADC1_ETRGREG_RM hands it to TIM8, and 10.2.11.8 says that
// remap exists only where the part has a TIM8: no CH32V203 does, and
// neither does the 128 KB CH32V303 - refused as a constant.
#include "ch32vx03/adc.hpp"

void f() { brio::Adc<1>::trigger<brio::AdcTrigger::tim8_trgo>(); }
