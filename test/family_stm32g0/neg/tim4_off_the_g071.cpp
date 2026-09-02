// mcu: stm32g071xx stm32g031xx
// TIM4 is bonded on the G0B1/G0C1 class alone (the header declares no
// TIM4_BASE elsewhere): a Tim<4> must not compile on the smaller parts.
#include "stm32g0/tim.hpp"
brio::Tim<4> t;
