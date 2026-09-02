// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 65 MHz exceeds the 64 MHz SYSCLK ceiling: no PLL ratio may reach it.
#include "stm32g0/clock.hpp"
using namespace brio;
void f() { (void)Clock<ClockSource::pll, 65'000'000>::init(); }
