// Two engines of one host must name DIFFERENT DMA channels: a transmit
// run and a receive run on one channel would each abort the other. On
// this chip a request is a FIELD any channel takes, so the channel is
// the identity and the pair below is the same channel twice.
#include "rp2350/dma.hpp"
#include "rp2350/spi.hpp"
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bad = brio::SpiHost<0, pins, brio::DmaTxEngine<4>, brio::DmaRxEngine<4>>;
void f() { (void)sizeof(Bad); }
