// The data phase is full duplex and its completion is the RECEIVE
// block's, so a host names BOTH DMA engines or neither: one engine alone
// must not compile.
#include "rp2350/dma.hpp"
#include "rp2350/spi.hpp"
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bad = brio::SpiHost<0, pins, brio::DmaTxEngine<4>, brio::NoDmaEngine>;
void f() { (void)sizeof(Bad); }
