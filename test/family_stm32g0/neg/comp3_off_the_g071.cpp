// mcu: stm32g071xx stm32g031xx
// 18.1: COMP3 is the STM32G0B1xx/G0C1xx's alone, and the smaller
// headers declare no COMP3_BASE - so a Comp<3> must not compile there.
#include "stm32g0/comp.hpp"
brio::Comp<3> c;
