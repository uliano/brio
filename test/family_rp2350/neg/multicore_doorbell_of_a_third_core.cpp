// Two cores per architecture, 0 and 1: there is no third bell.
#include "rp2350/multicore.hpp"

void f() { brio::SioDoorbell<2>::ring(); }
