// mcu: ch32v006k8
// This family has I2C1 alone: a second instance must be REFUSED.
#include "ch32v00x/i2c.hpp"

void f() { brio::I2c<2>::enable(); }
