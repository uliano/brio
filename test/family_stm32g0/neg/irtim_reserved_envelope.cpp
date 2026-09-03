// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// SYSCFG_CFGR1.IR_MOD code 11 is Reserved (RM0444 6.1.3 / ch. 27).
#include "stm32g0/irtim.hpp"
static_assert(brio::irtim_envelope_valid(static_cast<brio::IrtimEnvelope>(3)));
void f() {}
