// mcu: ch32v006k8 ch32v003f4
// The trampoline has no primary template: a third tag names an incomplete
// type where it is written.
#include "util/placement.hpp"

struct InTightlyCoupled {};
int f() { return brio::Hot<InTightlyCoupled>::run([] { return 1; }); }
