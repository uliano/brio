// mcu: ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8
// I2C1'S SECOND COLUMN (table 10-34, AFIO_PCFR1's I2C1 bit) puts the
// bus on PB8/PB9, and these packages bring out PB9 nowhere - so the
// column is not a remap here but a disconnection, and a host named on
// it is refused at compile time.
#include "ch32v203/i2c.hpp"

using Moved = brio::I2cHost<1, brio::i2c_pins_for(1, 1)>;
void f() { (void)Moved::status(); }
