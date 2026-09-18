// Must FAIL: this chip carries two I2C controllers (12.2), so there is
// no instance 2 - and no pin set can name one either.
#include "rp2350/i2c.hpp"

void up() { (void)brio::DwApbI2c<2>::enabled(); }
