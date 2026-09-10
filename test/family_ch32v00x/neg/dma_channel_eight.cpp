// mcu: ch32v006k8
// The CH32V00x DMA has channels 1..7: an eighth must be REFUSED at
// compile time, not addressed past the block.
#include "ch32v00x/dma.hpp"

void f() { brio::DmaChannel<8>::stop(); }
