// mcu: avr64ea48 avr64ea28
// VDDDIV10 and TEMPSENSE are positive inputs only (DS40002443A 31.5.14):
// a differential pair naming one as its negative is refused by AdcDiff.
#include "avrxt/adc.hpp"
using namespace brio;
void f() { Adc<0>::select(AdcDiff<AnalogIn<Pin<'D', 1>>{}, AdcInput::temp>{}); }
