// mcu: stm32g071xx stm32g031xx
// USART5 and USART6 are the G0B1/G0C1 class's.
#include "stm32g0/usart.hpp"
using namespace brio;
void f() { Usart<5>::bus_clock(true); }
