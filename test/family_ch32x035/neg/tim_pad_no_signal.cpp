// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// TIM1's codes 1xx carry CH3 and ETR alone (8.3.2.1): CH1 there is no pad.
#include "ch32x035/tim.hpp"

bool f() { return brio::TimPad<brio::afio_tim1_pads(4).ch1>::claim(); }
