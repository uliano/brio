// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE CH32V303'S DMA1 HAS NO EIGHTH CHANNEL. RM 11.3.1's notes give
// channel 8 to five classes and the CH32V30x_D8 is not among them - its
// DMA1 has seven, with a second controller of eleven beside it - so the
// channel every CH32V203 has is refused here on the line that named it.
#include "ch32vx03/dma.hpp"

using Eighth = brio::DmaChannel<1, 8>;
void f() { Eighth::stop(); }
