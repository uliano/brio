// REFUSED: a thirteenth PWM slice. The block has twelve (12.5.1), four
// more than the RP2040's, and the twelfth is the last.
#include "rp2350/pwm.hpp"

using namespace brio;

void slice() { (void)PwmSlice<12>::counter(); }
