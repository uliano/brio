// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A LINE THIS CLASS'S TABLE HAS NOT GOT. The eighth DMA1 channel's vector
// is the CH32V203's (entry 62 or 67 by class); the CH32V303's table has
// none, so the name is refused where it is used.
#include "ch32v203/device.hpp"

void f() { (void)brio::irq_present<brio::Irq::dma1_channel8>(); }
