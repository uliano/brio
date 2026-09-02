// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// HSISYS is 16 MHz over a power of two; 12 MHz wants the PLL.
#include "stm32g0/clock.hpp"
using namespace brio;
void f() { (void)Clock<ClockSource::internal, 12'000'000>::init(); }
