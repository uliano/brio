// Must FAIL: an I2C host names both DMA engines or neither - a read
// phase needs the commands poured and the bytes collected.
#include "rp2350/dma.hpp"
#include "rp2350/i2c.hpp"

constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
void up() { (void)brio::I2cHost<0, pins, brio::DmaTxEngine<4, uint16_t>>::status(); }
