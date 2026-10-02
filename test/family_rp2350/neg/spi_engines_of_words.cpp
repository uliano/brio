// Must FAIL: the widest beat SSPDR takes is sixteen bits (12.3), so the
// engines of an SPI host carry uint8_t or uint16_t elements - a word
// engine would pour two frames into one register write.
#include "rp2350/dma.hpp"
#include "rp2350/spi.hpp"
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bad = brio::SpiHost<0, pins, brio::DmaTxEngine<4, uint32_t>, brio::DmaRxEngine<5, uint32_t>>;
void f() { (void)sizeof(Bad); }
