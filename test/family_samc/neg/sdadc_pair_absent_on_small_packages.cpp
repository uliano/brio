// mcu: samc21e18a samc21g18a
// Differential pair 2 is PB06/PB07, which only the J bonds. Selecting it
// on a smaller package asks the converter for pads that do not exist.

#include "samc/sdadc.hpp"

using namespace brio;

void use() { (void)Sdadc::select<2>(); }
