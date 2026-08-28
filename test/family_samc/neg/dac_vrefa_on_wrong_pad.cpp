// mcu: samc21e18a samc21g18a samc21j18a
// The external reference pin is PA03; PA02 is the analog OUTPUT.

#include "samc/dac.hpp"

using namespace brio;

void use() { Dac::claim_vrefa<Pin<'A', 2>>(); }
