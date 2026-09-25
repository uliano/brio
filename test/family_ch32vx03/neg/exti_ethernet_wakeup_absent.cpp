// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// EXTI line 19 is the Ethernet's wake-up (table 9-3): the CH32V203RB is
// the one part here with a MAC, and the CH32V303 is not one of the
// classes that carries one, so its part table leaves the line out.
#include "ch32vx03/exti.hpp"

using EthWake = brio::ExtiLine<19>;
void f() { (void)EthWake::arm(true); }
