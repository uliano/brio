// Must FAIL: a host names both DMA engines or neither.
#include "rp2040/dma.hpp"
#include "rp2040/spi.hpp"

constexpr brio::SpiPins host_pins{.sck = 18, .tx = 19, .rx = 16};
void up() { (void)brio::SpiHost<0, host_pins, brio::DmaTxEngine<4>, brio::NoDmaEngine>::status(); }
