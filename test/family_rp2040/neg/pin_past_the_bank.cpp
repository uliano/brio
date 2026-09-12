// The user bank ends at GPIO29: Pin<30> must not compile.
#include "rp2040/pin.hpp"
using Bad = brio::Pin<30>;
void f() { Bad::output(); }
