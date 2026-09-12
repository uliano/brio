// Must FAIL: the transmit engine of an I2C host carries uint16_t
// command entries, never bytes (a narrow write is replicated across
// IC_DATA_CMD).
#include "rp2040/dma.hpp"
#include "rp2040/i2c.hpp"

constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
void up() { (void)brio::I2cHost<0, pins, brio::DmaTxEngine<4>, brio::DmaRxEngine<5>>::status(); }
