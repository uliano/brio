// Line 19 is the Ethernet wake-up: it exists on the parts with an
// Ethernet MAC and nowhere else, so naming it as a constant must be
// refused on a part without one.
// mcu: stm32f446xx stm32f411xe stm32f401xc stm32f412zx
#include "stm32f4/exti.hpp"
void f() { (void)brio::ExtiLine<19>::arm(true); }
