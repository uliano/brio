// mcu: ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// Locking a pin the package does not bond says nothing: a constant mask
// naming PA8 is refused where PA8 is not a pin.
#include "ch32x035/pin.hpp"

void f() { (void)brio::Port<'A'>::lock<(1UL << 8)>(); }
