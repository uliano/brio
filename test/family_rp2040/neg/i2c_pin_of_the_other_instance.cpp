// Must FAIL: I2C0's SDA is not on GPIO 14 (that is I2C1's), table 279.
#include "rp2040/i2c.hpp"

constexpr brio::I2cPins wrong{.scl = 13, .sda = 14};
void up() { (void)brio::I2cHost<0, wrong>::status(); }
