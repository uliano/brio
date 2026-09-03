// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// This family has at most three comparators (18.1) and the device header
// declares no COMP4_BASE anywhere.
#include "stm32g0/comp.hpp"
brio::Comp<4> c;
