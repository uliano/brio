// A UART pin is handed over under function 2 and nothing else.
#include "rp2040/uart.hpp"
constexpr brio::UartPins p{.tx = {0, brio::PinFunction::sio}, .rx = {1, brio::PinFunction::uart}};
using Bad = brio::Uart<0, p>;
void f() { (void)Bad::write_byte(0); }
