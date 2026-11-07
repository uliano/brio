// mcu: ch32v006f8 ch32v006d8 ch32v003f4 ch32v003a4 ch32v003j4
// PA0 is bonded by the CH32V006's QFN32 and QSOP24 alone (DS V2.0 table
// 2-1-1) and by no package of the CH32V003 (DS V1.6 table 2-1): a Pin on
// it must be REFUSED where the package leaves it out.
#include "ch32v00x/pin.hpp"

using Missing = brio::Pin<'A', 0>;
void f() { Missing::output(); }
