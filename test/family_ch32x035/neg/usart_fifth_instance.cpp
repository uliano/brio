// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// The series has USART1..USART4 (RM ch. 14): a fifth is refused.
#include "ch32x035/usart.hpp"

void f() { brio::Usart<5>::bus_clock(true); }
