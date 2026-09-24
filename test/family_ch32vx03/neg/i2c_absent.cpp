// mcu: ch32v203f6
// NO I2C ON THIS PART (datasheet table 2-1): the 32 KB TSSOP20 bonds
// neither PB6 nor PB7, which is where I2C1's only fully bonded column
// lives, and the part table says i2c_count = 0. The instance must be
// refused rather than answered at an address whose pads go nowhere.
#include "ch32vx03/i2c.hpp"

using Only = brio::I2c<1>;
void f() { Only::bus_clock(true); }
