// There are two GPIO clock inputs, GPIN0 and GPIN1.
#include "rp2350/clock.hpp"
using Bad = brio::ClockIn<2>;
void f() { (void)Bad::init(); }
