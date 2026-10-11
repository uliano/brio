// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// The circular shape needs a ring of at least two items (a lap of one
// has no position to read): refused.
#include "ch32x035/dma.hpp"

volatile uint8_t data;
uint8_t ring[1];
void f() { (void)brio::DmaRxEngine<8>::arm_ring(&data, ring); }
