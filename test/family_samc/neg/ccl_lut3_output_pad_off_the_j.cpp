// mcu: samc21e18a samc21g18a
// LUT3'S PADS ARE THE J'S ALONE: IN[9..11] on PB14/PB15/PB16 and OUT[3]
// on PB17. On the E and the G the LUT exists (CCL_LUT_NUM is 4
// everywhere) and is reachable only through events, a link or a
// sequencer - so the pad, and not the LUT, is what must be refused.

#include "samc/ccl.hpp"

using namespace brio;

void use() { CclOut<Pin<'B', 17>>::claim(); }
