// REFUSED: a third PWM interrupt line. This chip added a SECOND one
// (12.5.1.1) - IRQ0 and IRQ1, each with its own enable, force and status
// register over the one raw register - and there is no third.
#include "rp2350/pwm.hpp"

using namespace brio;

void line() { Pwm::interrupts<2>(PwmSlice<0>::bit, true); }
