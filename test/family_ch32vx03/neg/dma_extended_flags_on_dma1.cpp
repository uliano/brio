// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE EXTENDED FLAG PAIR IS DMA2'S. DMA2_EXTEM_INTFR (RM 11.3.11) holds
// the flags of DMA2's channels 8..11, which INTFR has no room for; DMA1's
// channels all report in its own INTFR, so asking DMA1 for the extended
// pair is refused on every part.
#include "ch32vx03/dma.hpp"

void f() { (void)brio::Dma<1>::extended_flags(); }
