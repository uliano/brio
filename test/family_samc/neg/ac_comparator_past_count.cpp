// mcu: samc21e18a samc21g18a samc21j18a
// The SAM C21 AC implements four comparators; a fifth must be refused
// at compile time on every variant.

#include "samc/ac.hpp"

void bad() { brio::AcComparator<4>::scaler(0); }
