// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb
// NO CH32V203 HAS A BASIC TIMER, and neither 128 KB CH32V303: chapter 16
// describes TIM6 and TIM7, which the CH32V203 datasheet's table 2-1
// gives to no part and the CH32V303's table 2-1-1 to the 256 KB two.
#include "ch32vx03/tim.hpp"

using Basic = brio::Tim<6>;
void f() { Basic::init(); }
