// Must FAIL: a pair is the A and the B of ONE slice (GPIO 12 is 6A,
// GPIO 15 is 7B).
#include "rp2040/pwm.hpp"

void up() { (void)brio::PwmPair<12, 15>::dead_time(); }
