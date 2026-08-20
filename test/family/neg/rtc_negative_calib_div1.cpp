// mcu: avr128db48 avr128da28
// A negative crystal error correction needs the prescaler at DIV2 or
// slower (26.6): the compile-time config form must refuse DIV1.
#include "avrdx/rtc.hpp"
using namespace brio;
void f() { Rtc::init<RtcConfig{.prescaler = RtcPrescaler::div1, .correction_ppm = -100}>(); }
