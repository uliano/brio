// mcu: samc21e18a samc21g18a samc21j18a
// PORT has exactly TWO groups on every SAM C21 variant (the device
// header's PORT_GROUPS says 2, and no variant in the pack declares a
// third). A 'C' group is refused everywhere, not on small packages
// only - which is what makes this negative name all three variants.
#include "samc/pin.hpp"
using namespace brio;
void f() { Pin<'C', 0>::output(); }
