// mcu: ch32v006k8
// The family has ports A..D: a port letter outside them is refused.
#include "ch32v00x/pin.hpp"

using Ghost = brio::Pin<'E', 0>;
void f() { Ghost::output(); }
