// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The CAN subsystem is TWO modules on the parts that have one (figure
// 392) and there is no third anywhere in the family.
#include "stm32g0/fdcan.hpp"
brio::Fdcan<3> absent;
