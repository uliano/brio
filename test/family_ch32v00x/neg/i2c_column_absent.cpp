// mcu: ch32v006f8 ch32v006d8
// I2C1's column 3 is PB5/PB6, absent from the CH32V006's 20- and 12-pin
// packages: the host must be REFUSED there.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/i2c.hpp"

using Host = brio::I2cHost<1, brio::i2c1_pins_for(3)>;
void f() { (void)Host::isr(); }
