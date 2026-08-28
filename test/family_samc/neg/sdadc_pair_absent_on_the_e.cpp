// mcu: samc21e18a
// Differential pair 1 is PB08/PB09, and the E bonds NO PORT B pad to the
// SDADC at all - so pair 0 is the only input that package has.

#include "samc/sdadc.hpp"

using namespace brio;

void use() { (void)Sdadc::select<1>(); }
