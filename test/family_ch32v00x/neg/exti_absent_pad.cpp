// mcu: ch32v006f8 ch32v006d8
// An external interrupt line on PB0 must be REFUSED where the package
// bonds no port B.
#include "ch32v00x/exti.hpp"

using Line = brio::ExtInt<'B', 0>;
void f() { Line::init(true, false); }
