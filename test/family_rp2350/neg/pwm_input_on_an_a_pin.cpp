// REFUSED: an edge counter on an A pin. Only a B pin is an input
// (12.5.2.5, table 1130), so a counter's pad is an ODD GPIO; GP12 is
// slice 6's A output.
#include "rp2350/pwm.hpp"

using namespace brio;

void counter() { (void)PwmEdgeCounter<12>::setup(); }
