// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// THE SECOND CONTROLLER ON A PART THAT HAS ONE. RM 11.2.3 gives DMA2 and
// its eleven channels to the V4F and Cortex-M3 classes - the CH32V303
// among them - and every CH32V203 carries one controller of eight, so a
// channel of DMA2 is refused on the line that named it.
#include "ch32vx03/dma.hpp"

using Second = brio::DmaChannel<2, 1>;
void f() { Second::stop(); }
