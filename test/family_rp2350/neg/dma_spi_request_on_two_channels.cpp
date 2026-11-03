// Must FAIL: one request, one channel, on the SPI host's claim: SPI0 on channels
// 4 and 5, then on 6 and 7. Two ELEMENT widths on the SAME two channels are one
// owner and would pass; the same request on another channel does not.
#include "rp2350/dma.hpp"
#include "rp2350/spi.hpp"
constexpr brio::SpiPins s0{.sck = 18, .tx = 19, .rx = 16};
using A = brio::SpiHost<0, s0, brio::DmaTxEngine<4, uint16_t>, brio::DmaRxEngine<5, uint16_t>>;
using B = brio::SpiHost<0, s0, brio::DmaTxEngine<6, uint16_t>, brio::DmaRxEngine<7, uint16_t>>;
void f() { (void)sizeof(A); (void)sizeof(B); }
