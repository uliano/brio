// mcu: ch32v006f8 ch32v006d8
// PD7 shares its package pin with PA4 on the CH32V006's 20- and 12-pin
// packages (DS V2.0 table 2-1-1, note 4): handing it to a peripheral
// output - TIM2_CH4, its default function - must be REFUSED.
#include "ch32v00x/pin.hpp"

using Shared = brio::Pin<'D', 7>;
void f() { Shared::function(); }
