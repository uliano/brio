// mcu: ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// A pin the package does not bond: PA8 is a pad of the die and a pin of
// the two LQFP packages alone.
#include "ch32x035/pin.hpp"

void f() { brio::Pin<'A', 8>::output(); }
