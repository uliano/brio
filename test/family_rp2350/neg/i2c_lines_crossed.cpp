// Must FAIL: the two lines of an I2C are not interchangeable - SDA is
// the EVEN pad of a pair and SCL the odd one, so naming GP13 as SDA and
// GP12 as SCL asks for the pads the other way round and does not
// compile.
#include "rp2350/i2c.hpp"

constexpr brio::I2cPins crossed{.scl = 12, .sda = 13};
void up() { (void)brio::I2cHost<0, crossed>::status(); }
