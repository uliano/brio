// The same refusal one package down: GP40 is bonded on the QFN-80 and
// not on the QFN-60, so an interrupt on it is a compile error in a build
// that states the smaller package.
#include "rp2350/pin.hpp"
using Bad = brio::ExtInt<brio::Pin<40>>;
void f() { (void)Bad::arm(brio::pin_edges); }
