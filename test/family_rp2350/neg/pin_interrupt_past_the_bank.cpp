// A pin interrupt is named by its PAD, so a pad the die has not got is
// refused where the pad is.
#include "rp2350/pin.hpp"
using Bad = brio::ExtInt<brio::Pin<48>>;
void f() { (void)Bad::arm(brio::pin_edges); }
