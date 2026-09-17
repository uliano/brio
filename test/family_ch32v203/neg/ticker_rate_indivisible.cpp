// mcu: ch32v203c8
// The tick rate must divide 1000 exactly, or millis() would drift: 300
// ticks per second must be REFUSED.
#include "ch32v203/ticker.hpp"

using Odd = brio::BasicTicker<300>;
void f() { (void)Odd::millis(); }
