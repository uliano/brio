// Memory to memory is DMA2's alone: DMA1's peripheral port - the source
// of such a block - is not connected to the bus matrix, so a copy engine
// on DMA1 would read nothing a memory holds.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
using namespace brio;
void f() { DmaCopyEngine<1, 0>::arm(); }
