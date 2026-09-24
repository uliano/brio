// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A crystal is named WITH its rate: the third parameter is not optional
// for an external root, because nothing on the chip can measure it.
#include "ch32vx03/clock.hpp"

using Unnamed = brio::Clock<brio::ClockSource::crystal, 8'000'000>;
void f() { (void)Unnamed::init(); }
