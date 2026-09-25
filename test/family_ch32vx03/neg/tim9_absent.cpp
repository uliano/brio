// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb
// TIM9 - like TIM8 and TIM10 - is one of the three advanced-control
// timers the 256 KB CH32V303 adds: table 2-1-1 counts ONE advanced timer
// on the CH32V303CB and RB and four on the RC and VC, and no CH32V203
// has more than TIM1. The part's mask is what refuses it.
#include "ch32vx03/tim.hpp"

using Third = brio::Tim<9>;
void f() { Third::init(); }
