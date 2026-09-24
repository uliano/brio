// mcu: ch32v203f6
// The smallest package offers ONE usart and it is USART2: neither
// PA9/PA10 nor the remapped PB6/PB7 is bonded here, so USART1 - the
// instance a count read as "the first n" would offer - must be refused.
#include "ch32vx03/usart.hpp"

using First = brio::Usart<1>;
void f() { First::bus_clock(true); }
