// mcu: samc21e18a samc21g18a samc21j18a
// VOUT is PA02 on this family and nothing else is; PA03 is the external
// reference pin, which is a different claim entirely.

#include "samc/dac.hpp"

using namespace brio;

void use() { Dac::claim_vout<Pin<'A', 3>>(); }
