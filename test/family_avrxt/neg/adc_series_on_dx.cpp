// mcu: avr128db48 avr128da48 avr128db28
// Series accumulation is the AVR EA's (DS40002443A table 31-1): the
// DA/DB's AdcConfig has no such knob.
#include "avrxt/adc.hpp"
using namespace brio;
AdcConfig f() { return AdcConfig{.accumulate = 4, .series = true}; }
