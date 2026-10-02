// The copy engine reporting on a third DMA interrupt line: the RP2040 has two.
#include "rp2040/dma.hpp"
void third() { brio::DmaCopyEngine<8, 2>::arm(); }
