// mcu: stm32g031xx
// The G031 class stops at five channels (the header declares no
// DMA1_Channel6_BASE), where the G071 and G0B1 classes have seven - so
// the same spelling that is legal there must be refused here.
#include "stm32g0/dma.hpp"
using Six = brio::DmaChannel<1, 6>;
static_assert(Six::index == 6);
