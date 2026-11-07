// mcu: ch32v006f8 ch32v006d8
// The CH32V006's die has port B, its 20- and 12-pin packages bond none of
// it (DS V2.0 table 2-1-1): a Pin on PB0 must be REFUSED there, even as
// an input.
#include "ch32v00x/pin.hpp"

using Missing = brio::Pin<'B', 0>;
void f() { Missing::input(); }
