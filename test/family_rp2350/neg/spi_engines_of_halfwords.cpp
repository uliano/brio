// The engined path serves 8-bit frames and the Request's buffers are
// BYTES, so a host whose engines carry 16-bit elements must not compile
// - a 16-bit request falls back to the pump instead.
#include "rp2350/dma.hpp"
#include "rp2350/spi.hpp"
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bad = brio::SpiHost<0, pins, brio::DmaTxEngine<4, uint16_t>, brio::DmaRxEngine<5, uint16_t>>;
void f() { (void)sizeof(Bad); }
