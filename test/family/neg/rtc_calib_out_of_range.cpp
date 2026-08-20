// mcu: avr128db48 avr128da28
// CALIB carries seven bits of magnitude: -128 ppm is not expressible.
#include "avrdx/rtc.hpp"
using namespace brio;
void f() { Rtc::init<RtcConfig{.prescaler = RtcPrescaler::div2, .correction_ppm = -128}>(); }
