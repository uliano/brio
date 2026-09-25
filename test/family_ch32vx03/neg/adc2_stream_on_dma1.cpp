// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE SECOND CONVERTER'S REQUEST ON THE FIRST CONTROLLER. Table 11-3 puts
// ADC2's request on DMA2's channel 5; DMA1's channel 5 is USART1's
// receiver, I2C2's and TIM1's update, and an engine there would never see
// a conversion.
#include "ch32vx03/adc.hpp"
#include "ch32vx03/dma.hpp"

using Wrong = brio::DmaPingPongEngine<1, 5, uint16_t>;
void f() { brio::Adc<2>::claim_stream<Wrong>(); }
