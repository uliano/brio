// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// Channels 1..3 of the advanced timer have a complementary output;
// channel 4 has not (RM 14.3.6, and CCER has no CC4NE bit).
#include "ch32vx03/tim.hpp"

using Pair = brio::TimPairPwm<brio::Tim<1>, 3, 1000>;
void f() { (void)Pair::setup(0, 0); }
