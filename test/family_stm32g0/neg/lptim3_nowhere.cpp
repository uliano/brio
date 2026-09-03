// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// No part of this family has a third low-power timer: the device header
// declares LPTIM1 and LPTIM2 and nothing else.
#include "stm32g0/lptim.hpp"
brio::Lptim<3> absent;
