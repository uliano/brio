// GP40 is bonded in the QFN-80 and not in the QFN-60, and a build that
// states its package refuses the pad instead of driving nothing.
#include "rp2350/pio.hpp"
using Bad = brio::PioSquareWave<0, 0, 40>;
void f() { Bad::release(); }
