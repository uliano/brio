// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// EXTI line 21 is the internal 32 kHz calibration's wake-up, which table
// 9-3 gives to the CH32V20x_D8 and D8W alone - the CH32V203RB here.
#include "ch32v203/exti.hpp"

using Calibration = brio::ExtiLine<21>;
void f() { (void)Calibration::arm(true); }
