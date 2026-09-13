// Every part of this family that carries the Fast-mode Plus I2C carries
// exactly ONE of it, and the index is there for the day a part arrives
// with two - not for a second instance to be named on a part with one.
// mcu: stm32f446xx stm32f412zx stm32f410rx
#include "stm32f4/fmpi2c.hpp"
void f() { (void)brio::FmpI2c<2>::number; }
