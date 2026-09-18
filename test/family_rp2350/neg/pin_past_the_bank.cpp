// The user bank ends at GPIO47 on the largest package: Pin<48> must not
// compile, whichever package the build is for.
#include "rp2350/pin.hpp"
using Bad = brio::Pin<48>;
void f() { (void)Bad::output(); }
