// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// Every timer of this family has FOUR capture/compare channels, and a
// task that names a fifth is asking for a register that is not there
// (RM 14.4.7, 15.4.7 - two control registers, two channels each).
#include "ch32v203/tim.hpp"

using Beyond = brio::TimPwm<brio::Tim<3>, 4, 1000>;
void f() { (void)Beyond::setup(0); }
