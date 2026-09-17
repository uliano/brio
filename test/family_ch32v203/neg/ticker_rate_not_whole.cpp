// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A tick is a whole number of milliseconds, so that millis() is one
// multiplication and not a division.
#include "ch32v203/ticker.hpp"

using Odd = brio::BasicTicker<3>;
void f() { (void)Odd::ticks(); }
