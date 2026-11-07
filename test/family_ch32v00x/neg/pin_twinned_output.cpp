// mcu: ch32v006e8 ch32v006f8 ch32v006d8 ch32v003j4
// A pad that shares its package pin with another (the CH32V006's notes 3
// and 4: PA1 with PA6, PA4 with PD7; the CH32V003's SOP8: PA1 with PD6)
// must be REFUSED as an output.
#include "ch32v00x/pin.hpp"

using Shared = brio::Pin<'A', brio::device::package == brio::Ch32Package::pins24 ? 6 :
                              brio::device::package == brio::Ch32Package::pins8 ? 1 : 4>;
static_assert(brio::pad_twinned(Shared::pad));
void f() { Shared::output(); }
