// mcu: avr64ea48 avr64ea32
// A comparator's DACREF is a negative input only on the AVR EA's ADC
// (DS40002443A 31.5.13 has no positive code for it).
#include "avrxt/adc.hpp"
using namespace brio;
void f() { Adc<0>::select(AdcDiff<AdcDacref::dacref0, AdcInput::gnd>{}); }
