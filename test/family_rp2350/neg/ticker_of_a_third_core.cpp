// Two cores per architecture, 0 and 1: there is no third ticker.
#include "rp2350/ticker.hpp"
using Bad = brio::CoreTicker<2>;
void f() { (void)Bad::ticks(); }
