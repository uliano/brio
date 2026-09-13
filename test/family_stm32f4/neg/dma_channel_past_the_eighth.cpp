// CHSEL is three bits: a stream chooses between eight request lines
// (RM0090 10.3.3), and a ninth is refused at the engine that named it.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
using namespace brio;
void f() { (void)DmaRxEngine<2, 2, 8>::idle(); }
