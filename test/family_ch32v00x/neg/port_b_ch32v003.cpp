// mcu: ch32v003f4
// The CH32V003 bonds ports A, C and D: a Pin on port B must be REFUSED
// on it (device::has_port_b through gpio_base_for).
#include "ch32v00x/pin.hpp"

using Missing = brio::Pin<'B', 0>;
void f() { Missing::output(); }
