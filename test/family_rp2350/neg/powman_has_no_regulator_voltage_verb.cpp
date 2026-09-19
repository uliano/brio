// The core regulator's output voltage is READ-ONLY in this tree: a wrong
// VSEL is a voltage on the one digital core supply, and VREG_CTRL.UNLOCK
// cannot be undone once it is set.
#include "rp2350/powman.hpp"
void f() { brio::Powman::vreg_vsel(0x0Bu); }
