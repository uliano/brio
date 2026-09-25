// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE DAC'S REQUEST NAMED ON THE WRONG CONTROLLER. DAC1 raises its request
// on DMA2's channel 3 (RM table 11-3); DMA1 has a channel 3 too, which is
// SPI1's transmitter and USART3's receiver, and an engine there would wait
// for a trigger that never reaches it.
#include "ch32vx03/dac.hpp"
#include "ch32vx03/dma.hpp"

using Wrong = brio::DmaLoopEngine<1, 3, uint16_t>;
void f() { brio::Dac::claim_stream<1, Wrong>(); }
