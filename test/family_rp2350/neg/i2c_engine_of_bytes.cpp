// Must FAIL: the transmit engine of an I2C host carries uint16_t command
// entries, never bytes - IC_DATA_CMD is a command word and a narrow
// write is replicated across it (2.1.5).
#include "rp2350/dma.hpp"
#include "rp2350/i2c.hpp"

constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
void up() { (void)brio::I2cHost<0, pins, brio::DmaTxEngine<4>, brio::DmaRxEngine<5>>::status(); }
