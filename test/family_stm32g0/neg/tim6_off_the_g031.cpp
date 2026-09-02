// mcu: stm32g031xx
// The basic timers TIM6/TIM7 are absent from the G031 class - the one
// place in this family where a TIME BASE with no channel is missing.
#include "stm32g0/tim.hpp"
brio::Tim<6> t;
