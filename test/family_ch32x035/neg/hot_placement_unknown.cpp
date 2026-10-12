// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// The trampoline has no primary template: a third tag names an incomplete
// type where it is written.
#include "util/placement.hpp"

struct InTightlyCoupled {};
int f() {
    return brio::Hot<InTightlyCoupled>::run([] { return 1; });
}
