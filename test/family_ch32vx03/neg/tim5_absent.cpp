// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v303cb ch32v303rb
// TIM5 belongs to the 128 KB CH32V203 (the CH32V203 datasheet's table
// 2-1, "General-purpose (32-bit)") and to the 256 KB CH32V303 (sixteen
// bits there, table 2-1-1), so every other part refuses it - and the
// part's own table is what says so.
#include "ch32vx03/tim.hpp"

using Wide = brio::Tim<5>;
void f() { Wide::init(); }
