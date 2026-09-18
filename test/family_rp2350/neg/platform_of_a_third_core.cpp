// Two cores per architecture, 0 and 1: there is no third platform.
#include "rp2350/platform.hpp"
using Bad = brio::Rp2350Platform<2>;
void f() { Bad::idle(); }
