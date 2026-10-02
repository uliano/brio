// Must FAIL: the two engines of an SPI host carry the SAME element - a
// data phase is full duplex, one frame out for every frame back, so a
// half-word transmit beside a byte receive would serve neither width.
#include "rp2350/dma.hpp"
#include "rp2350/spi.hpp"
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bad = brio::SpiHost<0, pins, brio::DmaTxEngine<4, uint16_t>, brio::DmaRxEngine<5, uint8_t>>;
void f() { (void)sizeof(Bad); }
