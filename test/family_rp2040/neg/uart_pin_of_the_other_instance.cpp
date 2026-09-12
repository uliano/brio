// GPIO 4/5 are UART1's pads (table 279): a UART0 on them must not compile.
#include "rp2040/uart.hpp"
constexpr brio::UartPins p{.tx = {4, brio::PinFunction::uart}, .rx = {5, brio::PinFunction::uart}};
using Bad = brio::Uart<0, p>;
void f() { (void)Bad::write_byte(0); }
