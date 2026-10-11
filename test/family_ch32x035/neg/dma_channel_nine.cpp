// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// RM 9.1: eight channels. A ninth is refused.
#include "ch32x035/dma.hpp"

void f() { brio::DmaChannel<9>::stop(); }
