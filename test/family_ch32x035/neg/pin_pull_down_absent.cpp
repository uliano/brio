// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// Only PA0..PA15, PC16 and PC17 have a pull-down (RM ch. 8's opening):
// asking PC19 for one, where the pull is a constant, is refused.
#include "ch32x035/pin.hpp"

void f() { brio::Pin<'C', 19>::input<brio::PinPull::down>(); }
