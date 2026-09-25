// mcu: ch32x035g8u ch32x035g8r
// PB1 shares its package pin with PB5 on the two 28-pin parts (table 2-1,
// note 5): handing it to a peripheral as an alternate output is refused.
#include "ch32x035/pin.hpp"

void f() { brio::Pin<'B', 1>::function(); }
