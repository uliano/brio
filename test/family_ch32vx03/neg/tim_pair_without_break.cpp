// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The complementary outputs and the dead-time generator are the
// ADVANCED-CONTROL timer's (RM 14.3.6): a general-purpose timer has
// neither, so a complementary pair on one is refused.
#include "ch32vx03/tim.hpp"

using Pair = brio::TimPairPwm<brio::Tim<3>, 0, 1000>;
void f() { (void)Pair::setup(0, 0); }
