// mcu: ch32v303rc ch32v303vc
// A BASIC TIMER HAS NO CHANNEL (RM 16.2.2): TIM6's register map stops at
// the time base, so a channel verb on it names registers that are not
// there - a compile error on the two parts that have the timer, where the
// general-purpose timers' missing features only answer false.
#include "ch32vx03/tim.hpp"

void f() { (void)brio::Tim<6>::output_channel(0, {.mode = brio::TimOutputMode::pwm1}); }
