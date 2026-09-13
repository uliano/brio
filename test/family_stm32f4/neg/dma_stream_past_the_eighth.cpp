// A controller of this family has eight streams, numbered 0..7 (RM0090
// 10.3.5): a ninth is refused at the type and not at the address.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
using namespace brio;
void f() { (void)DmaStream<1, 8>::enabled(); }
