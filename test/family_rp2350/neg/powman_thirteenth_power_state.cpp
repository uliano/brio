// Table 475 has twelve states and no thirteenth: the switched core and
// three memory domains are four bits, and there is no P2.
#include "rp2350/powman.hpp"
void f() { (void)brio::Powman::power_down(brio::PowerState::p2_0); }
