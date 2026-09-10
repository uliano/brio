// mcu: ch32v006k8
// TIM1's channel 4 has no complementary output (11.1: three of them),
// so a pair on it must be REFUSED at compile time.
#include "ch32v00x/tim.hpp"

void f() { (void)brio::TimPairPwm<brio::Tim<1>, 3, 1000>::setup(); }
