// mcu: ch32v203rb ch32v303rc ch32v303vc
// TIM5 is a GENERAL-PURPOSE timer on every part that has it - thirty-two
// bits on the CH32V203RB, sixteen on the CH32V303RC and VC - and so has
// no break input and no dead-time generator (RM 15.2): the complementary
// pair, which needs both, is refused on it.
#include "ch32vx03/tim.hpp"

using Pair = brio::TimPairPwm<brio::Tim<5>, 0, 1000>;
void f() { (void)Pair::setup(0, 0); }
