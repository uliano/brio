// mcu: stm32g071xx stm32g031xx
// LPUART2 is the G0B1/G0C1 class's alone (RM0444 table 183).
#include "stm32g0/lpuart.hpp"
void f() { brio::Lpuart<2>::bus_clock(true); }
