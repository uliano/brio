// mcu: stm32g071xx stm32g031xx
// A second DMA controller exists on the G0B1/G0C1 class alone: the
// smaller headers declare no DMA2_BASE, so Dma<2> must not compile there.
#include "stm32g0/dma.hpp"
using Second = brio::Dma<2>;
static_assert(Second::channels > 0);
