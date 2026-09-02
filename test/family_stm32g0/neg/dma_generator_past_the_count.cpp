// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// Table 54: four request-generator channels on every part, numbered 0..3.
#include "stm32g0/dma.hpp"
using Fifth = brio::DmaMuxGenerator<4>;
static_assert(Fifth::index == 4);
