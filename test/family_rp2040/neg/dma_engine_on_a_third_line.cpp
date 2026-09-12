// Must FAIL: the RP2040 DMA has two interrupt lines; an engine on a
// third is refused.
#include "rp2040/dma.hpp"

void third() { brio::DmaTxEngine<0, uint8_t, 2>::arm(nullptr, brio::Dreq::permanent); }
