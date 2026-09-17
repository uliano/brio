// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// NO PART OF THIS SERIES HAS A BASIC TIMER: chapter 16 describes TIM6
// and TIM7, which the datasheet's table 2-1 gives to no CH32V203.
#include "ch32v203/tim.hpp"

using Basic = brio::Tim<6>;
void f() { Basic::init(); }
