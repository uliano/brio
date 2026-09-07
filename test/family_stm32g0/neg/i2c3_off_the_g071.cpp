// mcu: stm32g071xx stm32g031xx
// Table 165's footnote 2: I2C3 "applies to STM32G0B1xx and STM32G0C1xx
// devices only". Every smaller part declares no I2C3_BASE, and the
// resource refuses the instance rather than pointing at an address the
// device has not got.
#include "stm32g0/i2c.hpp"
using Third = brio::I2c<3>;
static_assert(sizeof(Third) >= 0, "instantiate it");
void use() { (void)Third::regs(); }
