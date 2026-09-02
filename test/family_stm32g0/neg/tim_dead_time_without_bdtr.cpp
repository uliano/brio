// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// TIM3 has no break/dead-time unit and therefore no complementary
// output: a complementary pair on it is a compile error, not a store
// into a register the silicon does not implement.
#include "stm32g0/tim.hpp"
using Bad = brio::TimPairPwm<brio::Tim<3>, 0, 100>;
Bad b;
