// mcu: stm32g031xx
// TIM15 exists from the G071 class up; the G031's header declares no
// TIM15_BASE, so its ITR entries and its two channels are unreachable.
#include "stm32g0/tim.hpp"
brio::Tim<15> t;
