// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE HIGH-SPEED READ AT BR = /4. 20.4.10 makes HSRXEN "only valid at
// clock division 2" on the CH32V20x_D6 and on some CH32V30x_D8 lots, and
// puts BR on a second ladder under it on others - so /2, the code both
// ladders share, is the only one at which the mode means the same thing
// on every die. A constant configuration asking it at /4 is refused.
#include "ch32vx03/spi.hpp"

using One = brio::Spi<1>;
void f() { One::configure_high_speed_read<brio::SpiConfig{.clock = brio::SpiClock::div4}>(); }
