// mcu: ch32v006k8
// A port of this family has eight pins: Pin<'A', 8> must be refused.
#include "ch32v00x/pin.hpp"

using Ghost = brio::Pin<'A', 8>;
void f() { Ghost::output(); }
