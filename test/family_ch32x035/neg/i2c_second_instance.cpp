// mcu: ch32x035r8 ch32x035f8
// The series has I2C1 alone (RM ch. 15, table 15-1).
#include "ch32x035/i2c.hpp"

void f() { brio::I2c<2>::bus_clock(true); }
