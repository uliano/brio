// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A BLOCK PLAYER ON A NINTH CHANNEL. The controller has eight, and the
// block engines are built on the same DmaChannel every other engine
// uses - so the refusal is the channel's and reaches the engine on the
// line that named it.
#include "ch32v203/dma.hpp"

using Beyond = brio::DmaLoopEngine<9, uint16_t>;
void f() { Beyond::stop(); }
