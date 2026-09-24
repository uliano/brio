// mcu: ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6
// ONE I2C ON THIS PART (datasheet table 2-1): every part of the series
// below the CH32V203C8 that has an I2C at all carries I2C1 alone, so
// the second instance must be refused rather than answered at an
// address nothing decodes.
#include "ch32vx03/i2c.hpp"

using Second = brio::I2c<2>;
void f() { Second::bus_clock(true); }
