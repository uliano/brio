// There are two system timers on this chip, TIMER0 and TIMER1.
#include "rp2350/timer.hpp"
using Bad = brio::Timer<2>;
void f() { (void)Bad::now_low(); }
