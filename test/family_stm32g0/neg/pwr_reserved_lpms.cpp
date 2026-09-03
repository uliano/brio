// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 4.4.1: LPMS 010 is Reserved. A code with no mode behind it must be
// refused before it reaches PWR_CR1, not written and hoped for.
#include "stm32g0/pwr.hpp"
static_assert(brio::pwr_mode_valid(static_cast<brio::PwrMode>(2)));
