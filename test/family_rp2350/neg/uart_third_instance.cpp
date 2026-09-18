// This chip carries two PL011s (datasheet 12.1): there is no UART2.
#include "rp2350/uart.hpp"
using Bad = brio::Pl011<2>;
void f() { (void)Bad::enabled(); }
