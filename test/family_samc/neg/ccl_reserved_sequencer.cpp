// mcu: samc21e18a samc21g18a samc21j18a
// SEQCTRLx.SEQSEL implements five values (DISABLE/DFF/JK/LATCH/RS);
// 0x5..0xF are Reserved (37.8.2).

#include "samc/ccl.hpp"

using namespace brio;

void use() { (void)Ccl::sequencer<0, static_cast<LutSequencer>(5)>(); }
