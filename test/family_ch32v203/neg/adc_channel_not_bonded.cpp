// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v303cb
// A CHANNEL THIS PACKAGE DOES NOT BRING OUT. ADC_IN10 is PC0 on every
// part of the family, and only the 64-pin and 100-pin ones bond port C's
// low pins - the others bring out PC13, PC14 and PC15 or no port C at
// all. The
// converter has the channel; the package has no pad for it, which is
// `Pin`'s own refusal and needs no table of its own in the driver.
#include "ch32v203/adc.hpp"

using Unbonded = brio::AnalogIn<brio::Pin<'C', 0>>;
void f() { Unbonded::claim(); }
