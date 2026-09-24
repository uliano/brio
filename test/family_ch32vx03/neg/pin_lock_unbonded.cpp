// mcu: ch32v203f6 ch32v203g6
// The configuration lock takes a MASK of pins, and these two packages
// bring out no PA8: locking a pin that is not there says nothing, so the
// constant face refuses the mask.
#include "ch32vx03/pin.hpp"

void f() { (void)brio::Port<'A'>::lock<0x0100>(); }
