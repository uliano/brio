// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE RIGHT CHANNEL NUMBER ON THE WRONG CONTROLLER. USART2 transmits on
// DMA1's channel 7 (table 11-2); DMA2 has a seventh channel too, and an
// engine there would be armed for a request that never comes - which is
// why a transport compares the whole slot, controller and channel.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

using P = brio::Ch32vx03Platform<>;
using Wrong = brio::Uart<2, P, 64, 64, brio::UartFormat{}, brio::DmaTxEngine<2, 7>>;
void f() { (void)Wrong::write_byte('x'); }
