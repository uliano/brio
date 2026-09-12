// Must FAIL: a PWM period of zero has no duty to set.
#include "rp2040/pio.hpp"

void up() { brio::PioPwm<0, 0, 17, 0>::duty(1); }
