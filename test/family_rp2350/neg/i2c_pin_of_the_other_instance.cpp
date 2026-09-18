// Must FAIL: I2C0's SDA is not on GP14 - that pad is I2C1's, and 9.4
// gives an I2C pad exactly one instance and one function.
#include "rp2350/i2c.hpp"

constexpr brio::I2cPins wrong{.scl = 13, .sda = 14};
void up() { (void)brio::I2cHost<0, wrong>::status(); }
