// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// DMA1 has seven channels at most on this family and they are numbered
// from ONE. Channel 8 is nobody's.
#include "stm32g0/dma.hpp"
using TooFar = brio::DmaChannel<1, 8>;
static_assert(TooFar::index == 8);
