// Port F is the 100-pin bondings' and up: a Pin on it must be refused on
// the parts whose header declares no GPIOF_BASE.
// mcu: stm32f411xe stm32f401xe stm32f410cx
#include "stm32f4/pin.hpp"
void f() { brio::Pin<'F', 0>::output(); }
