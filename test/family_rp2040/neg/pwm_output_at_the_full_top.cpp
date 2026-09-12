// Must FAIL: TOP 0xFFFF leaves no room for the 100 % level (TOP + 1).
#include "rp2040/pwm.hpp"

void up() { (void)brio::PwmOutput<12, 0xFFFF>::duty(); }
