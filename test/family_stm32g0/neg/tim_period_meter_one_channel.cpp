// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// PWM input mode watches ONE input with TWO channels (RM0444 21.3.6):
// TIM14 cannot do it, having one channel and no slave controller.
#include "stm32g0/tim.hpp"
using Bad = brio::TimPeriodMeter<brio::Tim<14>>;
Bad b;
