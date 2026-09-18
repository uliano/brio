// The pads are fixed per instance (datasheet 9.4): GP0/GP1 are UART0's,
// so a UART1 asked to use them must not compile.
#include "rp2350/uart.hpp"
constexpr brio::UartPins zero_pins{.tx = {0, brio::PinFunction::uart},
                                   .rx = {1, brio::PinFunction::uart}};
using Bad = brio::Uart<1, zero_pins>;
void f() { (void)sizeof(Bad); }
