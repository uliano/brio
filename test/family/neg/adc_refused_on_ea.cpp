// mcu: avr64ea48 avr64ea28
// The differential ADC of the AVR EA has no driver: adc.hpp refuses the part.
#include "avrdx/adc.hpp"
using namespace brio;
void f() {}
