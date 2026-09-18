// The QFN-60 brings out GPIO0..GPIO29: in a build that states that
// package, Pin<30> must not compile - and in a QFN-80 build it must,
// which is why this negative is the fixture's one `qfn60_` case.
#include "rp2350/pin.hpp"
using Bad = brio::Pin<30>;
void f() { (void)Bad::output(); }
