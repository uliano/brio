// mcu: ch32v006f8 ch32v006d8
// USART1's column 4 puts RX on PB3, which the CH32V006's 20- and 12-pin
// packages do not bond: the port must be REFUSED there.
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

using Port = brio::Uart<1, brio::Ch32v00xPlatform<>, 16, 16, brio::NoDmaEngine, brio::NoDmaEngine, 4>;
void f() { (void)Port::isr(); }
