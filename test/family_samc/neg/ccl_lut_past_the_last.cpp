// mcu: samc21e18a samc21g18a samc21j18a
// CCL_LUT_NUM is four on every member of this family (37.2), and the
// register file has four LUTCTRLn. A fifth would index past it.

#include "samc/ccl.hpp"

using namespace brio;

void use() { Lut<4>::enable(true); }
