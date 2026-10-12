// mcu: ch32x035f8 ch32x035f7
// TIM1's default column puts C2N on PB7, which the 20-pin CH32X035
// packages do not bond: the pad is refused at compile time.
#include "ch32x035/tim.hpp"

bool f() { return brio::TimPad<brio::afio_tim1_pads(0).ch2n>::claim(); }
