// The element type IS the access width, and SxCR.PSIZE has three codes:
// an engine over a type wider than a word has no width to name.
// mcu: stm32f429xx
#include <stdint.h>
#include "stm32f4/dma.hpp"
using namespace brio;
void f() { (void)DmaTxEngine<2, 7, 4, uint64_t>::busy(); }
