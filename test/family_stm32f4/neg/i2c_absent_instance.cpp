// I2C3 is on every part of this family but the F410, whose two instances
// are I2C1 and I2C2 (and an FMPI2C1 that is another block entirely).
// mcu: stm32f410rx stm32f410cx stm32f410tx
#include "stm32f4/i2c.hpp"
void f() { brio::I2c<3>::enable(); }
