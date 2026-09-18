// A PwmChannel counted by a program needs a period to count: at zero
// there is no duty to set.
#include "rp2350/pio.hpp"
using Bad = brio::PioPwm<0, 0, 13, 0>;
void f() { Bad::duty(0); }
