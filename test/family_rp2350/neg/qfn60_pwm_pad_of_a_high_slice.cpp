// REFUSED IN THE QFN-60: an output on GP32, slice 8's A pad. The four
// highest slices exist on both packages, but their pads are GPIO 32..47
// and the QFN-60 stops at GP29 (12.5.2's note, and device.hpp's package
// fact) - there they are repeating timers with no output, which is what
// PwmPeriodicTick is for.
#include "rp2350/pwm.hpp"

using namespace brio;

void output() { (void)PwmOutput<32, 999>::setup(); }
