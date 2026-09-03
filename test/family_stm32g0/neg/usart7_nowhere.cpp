// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The family stops at USART6 (RM0444 table 183): no device header of the
// pack declares a USART7_BASE, so the instance cannot exist anywhere.
#include "stm32g0/usart.hpp"
void f() { brio::Usart<7>::bus_clock(true); }
