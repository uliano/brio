// This family has TWO DMA controllers and the reserve reads their bases
// off the device header: a third is refused where it is named, not
// silently addressed at zero.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
using namespace brio;
void f() { (void)Dma<3>::bus_clock(); }
