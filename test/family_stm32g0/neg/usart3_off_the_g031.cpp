// mcu: stm32g031xx
// The G031 class has USART1 and USART2 only.
#include "stm32g0/usart.hpp"
using namespace brio;
void f() { Usart<3>::bus_clock(true); }
