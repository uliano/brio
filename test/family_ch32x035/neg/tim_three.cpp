// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// TIM3 is chapter 13's general-purpose block, not an advanced-control
// timer: Tim<3> is refused.
#include "ch32x035/tim.hpp"

void f() { brio::Tim<3>::init(); }
