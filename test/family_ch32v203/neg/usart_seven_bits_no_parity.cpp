// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// M COUNTS THE WORD AND NOT THE DATA (RM 18.10.4): eight or nine bits
// INCLUDING the parity bit, so seven data bits exist only with a parity
// bit. A frame of seven with none is a shape the register cannot hold.
#include "ch32v203/platform.hpp"
#include "ch32v203/usart.hpp"

using Bare7 = brio::Uart<2, brio::Ch32v203Platform<>, 64, 64,
                         brio::UartFormat{brio::UartBits::seven, brio::UartParity::none}>;
void f() { (void)Bare7::baud(); }
