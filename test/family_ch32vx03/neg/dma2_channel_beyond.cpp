// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A TWELFTH CHANNEL OF DMA2. Table 11-8 stops at DMA2_MADDR11 and the
// extended flag pair of 11.3.11 holds four channels, 8 to 11 - so the
// second controller of every CH32V303 has eleven, and a twelfth is a
// register that is not there.
#include "ch32vx03/dma.hpp"

using Beyond = brio::DmaChannel<2, 12>;
void f() { Beyond::stop(); }
