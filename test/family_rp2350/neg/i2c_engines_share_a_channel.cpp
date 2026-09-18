// Must FAIL: the two engines of one host must ride two different DMA
// channels - on this chip a request is a FIELD any channel takes, so the
// channel is the identity and the pair below is the same channel twice.
#include "rp2350/dma.hpp"
#include "rp2350/i2c.hpp"

constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
void up() {
    (void)brio::I2cHost<0, pins, brio::DmaTxEngine<4, uint16_t>, brio::DmaRxEngine<4>>::status();
}
