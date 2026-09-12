// Must FAIL: the RP2040 DMA has channels 0..11; a thirteenth is refused.
#include "rp2040/dma.hpp"

void thirteenth() { (void)brio::DmaChannel<12>::busy(); }
