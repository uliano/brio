// The Chrom-Art accelerator is on one class of this family; every other
// part's header declares no DMA2D_BASE and nothing of dma2d.hpp exists
// there.
// mcu: stm32f446xx stm32f411xe stm32f401xe stm32f412zx stm32f413xx
#include "stm32f4/dma2d.hpp"
void f() { brio::Dma2d::init(); }
