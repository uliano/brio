// The RP2040 has UART0 and UART1.
#include "rp2040/uart.hpp"
void f() { brio::Pl011<2>::enable(true); }
