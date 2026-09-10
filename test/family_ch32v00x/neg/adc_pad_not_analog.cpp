// mcu: ch32v006k8
// PC0 is no ADC input on the CH32V006 (the eight are PA2, PA1, PC4, PD2,
// PD3, PD5, PD6, PD4): an AnalogIn on it must be REFUSED.
#include "ch32v00x/adc.hpp"

using Bad = brio::AnalogIn<brio::Pin<'C', 0>>;
void f() { Bad::claim(); }
