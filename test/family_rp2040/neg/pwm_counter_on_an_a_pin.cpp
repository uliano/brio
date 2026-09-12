// Must FAIL: only a B pin is a PWM input (table 515: GPIO 14 is 7A).
#include "rp2040/pwm.hpp"

void up() { (void)brio::PwmEdgeCounter<14>::count(); }
