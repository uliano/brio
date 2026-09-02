// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// TIM16 has ONE capture/compare channel (DS13560 table 7): a task
// naming its channel 2 must fail to compile, where the resource's
// runtime verb would merely return false.
#include "stm32g0/tim.hpp"
using Bad = brio::TimPwm<brio::Tim<16>, 1, 100>;
Bad b;
