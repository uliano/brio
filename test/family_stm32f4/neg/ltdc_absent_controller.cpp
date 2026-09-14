// The display controller is on the parts with a panel interface alone.
// The two of the same class WITHOUT one carry the accelerator and the
// second PLL and no LTDC at all, and so does every smaller part.
// mcu: stm32f427xx stm32f437xx stm32f446xx stm32f411xe stm32f407xx
#include "stm32f4/ltdc.hpp"
void f() { brio::Ltdc::clock(true); }
