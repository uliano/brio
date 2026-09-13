// A GPIO port has sixteen pins.
// mcu: stm32f429xx stm32f411xe
#include "stm32f4/pin.hpp"
void f() { brio::Pin<'A', 16>::output(); }
