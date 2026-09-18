// Four address translation panes to a window: ATRANS0..3 for window 0,
// ATRANS4..7 for window 1.
#include "rp2350/flash.hpp"
void f() { (void)brio::QmiWindow<0>::translation<4>(); }
