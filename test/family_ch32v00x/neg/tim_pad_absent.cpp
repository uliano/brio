// mcu: ch32v006f8 ch32v006d8 ch32v003f4
// TIM1's column 9 takes CH1 on PA0, which these packages do not bond: the
// timer pad must be REFUSED.
#include "ch32v00x/tim.hpp"

using Missing = brio::TimPad<brio::Pad{'A', 0}>;
void f() { Missing::claim(); }
