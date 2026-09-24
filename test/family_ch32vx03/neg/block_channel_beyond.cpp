// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A BLOCK PLAYER ON A NINTH CHANNEL. DMA1 has eight or seven, and the
// block engines are built on the same DmaChannel every other engine
// uses - so the refusal is the channel's and reaches the engine on the
// line that named it.
#include "ch32vx03/dma.hpp"

using Beyond = brio::DmaLoopEngine<9, uint16_t>;
void f() { Beyond::stop(); }
