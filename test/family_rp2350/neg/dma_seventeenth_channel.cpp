// This chip's DMA has SIXTEEN channels (datasheet 12.6, four more than
// the RP2040's twelve): a seventeenth is not a channel, and naming one
// must not compile - the CHAIN_TO field is four bits wide and a bit map
// of channels is sixteen bits, so there is nowhere for it to go.
#include "rp2350/dma.hpp"
using Bad = brio::DmaChannel<16>;
void f() { (void)Bad::busy(); }
