// REFUSED: an output whose TOP is 0xFFFF. The 100 % level is TOP + 1
// (12.5.2.2) and a PwmChannel's `max` is that level, so a TOP of 0xFFFF
// would want a seventeen-bit level.
#include "rp2350/pwm.hpp"

using namespace brio;

void output() { (void)PwmOutput<13, 0xFFFF>::setup(); }
