// mcu: ch32v003a4
// The CH32V003's SOP16 leaves PC5 out (DS V1.6 table 2-1): a Pin on it
// must be REFUSED there.
#include "ch32v00x/pin.hpp"

using Missing = brio::Pin<'C', 5>;
void f() { Missing::input(); }
