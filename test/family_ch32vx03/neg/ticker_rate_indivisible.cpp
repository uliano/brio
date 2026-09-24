// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The tick rate must divide 1000 exactly, or millis() would drift: 300
// ticks per second must be REFUSED.
#include "ch32vx03/ticker.hpp"

using Odd = brio::BasicTicker<300>;
void f() { (void)Odd::millis(); }
