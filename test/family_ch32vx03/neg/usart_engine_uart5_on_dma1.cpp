// mcu: ch32v303rc ch32v303vc
// A DMA2 REQUEST NAMED ON DMA1. UART5 transmits on DMA2's channel 4
// (table 11-3); DMA1's fourth channel is USART1's transmitter and SPI2's
// receiver, and an engine there would never see UART5's request. The slot
// is the controller AND the channel, and the transport compares both.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

using P = brio::Ch32vx03Platform<>;
using Wrong = brio::Uart<5, P, 64, 64, brio::UartFormat{}, brio::DmaTxEngine<1, 4>>;
void f() { (void)Wrong::write_byte('x'); }
