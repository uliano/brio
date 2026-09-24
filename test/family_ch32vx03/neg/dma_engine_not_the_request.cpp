// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE CHANNEL IS THE REQUEST (RM 11.2.3): USART2 transmits on channel 7
// and on no other, so a transmit engine sitting on channel 4 - which is
// USART1's - would be armed for a request that never comes.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

using P = brio::Ch32vx03Platform<>;
using Wrong = brio::Uart<2, P, 64, 64, brio::UartFormat{}, brio::DmaTxEngine<4>>;
void f() { (void)Wrong::write_byte('x'); }
