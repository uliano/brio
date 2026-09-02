// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// TIM14, TIM16 and TIM17 have no slave controller (RM0444 24.4, 25.6),
// so nothing can make one count another timer's trigger.
#include "stm32g0/tim.hpp"
using Bad = brio::TimEventCounter<brio::Tim<17>>;
Bad b;
