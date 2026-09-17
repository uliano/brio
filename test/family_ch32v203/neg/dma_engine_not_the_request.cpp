// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// THE CHANNEL IS THE REQUEST (RM 11.2.3): USART2 transmits on channel 7
// and on no other, so a transmit engine sitting on channel 4 - which is
// USART1's - would be armed for a request that never comes.
#include "ch32v203/dma.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/usart.hpp"

using P = brio::Ch32v203Platform<>;
using Wrong = brio::Uart<2, P, 64, 64, brio::UartFormat{}, brio::DmaTxEngine<4>>;
void f() { (void)Wrong::write_byte('x'); }
