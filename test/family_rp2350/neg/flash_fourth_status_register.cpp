// Three status registers are readable on this class of chip: 05h, 35h
// and 15h.
#include "rp2350/flash.hpp"
void f() { (void)brio::Flash::status_register<4>(); }
