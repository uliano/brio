// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// This series addresses ports A, B and C: a port D is refused.
#include "ch32x035/pin.hpp"

void f() { brio::Pin<'D', 0>::output(); }
