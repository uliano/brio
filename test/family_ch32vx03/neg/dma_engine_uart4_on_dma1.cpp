// mcu: ch32v303rc ch32v303vc
// A DMA2 REQUEST NAMED ON DMA1. On the CH32V303 UART4 transmits on DMA2's
// channel 5 (table 11-3), where the CH32V203 has it on DMA1's channel 1;
// DMA1's fifth channel is USART1's receiver and I2C2's, so an engine
// there would never see UART4's request.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

using P = brio::Ch32vx03Platform<>;
using Wrong = brio::Uart<4, P, 64, 64, brio::UartFormat{}, brio::DmaTxEngine<1, 5>>;
void f() { (void)Wrong::write_byte('x'); }
