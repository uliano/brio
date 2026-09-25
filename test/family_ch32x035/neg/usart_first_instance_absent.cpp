// mcu: ch32x035f8 ch32x035f7
// The two 20-pin CH32X035 bond no USART1 column with both its TX and its
// RX pads, so the instance is not offered there and naming it is refused.
#include "ch32x035/usart.hpp"

void f() { brio::Usart<1>::bus_clock(true); }
